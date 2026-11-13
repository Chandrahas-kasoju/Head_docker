import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from vision_msgs.msg import Point2D

import pygame
import random
import sys
import math

# Tele's face: a patrol robot on duty, drawn with sharp, angular shapes.
#   On duty  : narrow slit eyes with a sensor bar that scans from side to side, a straight
#              mouth, a scan line and "ON PATROL" on the status strip. This is the face
#              whenever Tele is not greeting, also while someone is merely in view; then
#              the sensor bars follow that person.
#   Greeting : the person wants to interact -> the same face with a small smile: the
#              ends of the mouth lift, the eyes narrow slightly and the brows rise
# In Christmas mode the same face wears a Santa hat and beard.

# --- Configuration Settings ---
BACKGROUND_COLOR = (12, 14, 20)
FACE_COLOR = (0, 200, 255)
HALO_COLOR = (0, 70, 95)         # Neon glow under the bright shapes
DIM_COLOR = (0, 95, 125)         # Thin display lines
TEXT_COLOR = (200, 245, 255)
IS_TESTING_ON_LAPTOP = False
FPS = 30

# --- Geometry (pixels on a 1080 x 1080 screen, relative to the centre; scaled to the real screen) ---
EYE_X, EYE_Y = 245, -95
EYE_WIDTH = 270
EYE_HEIGHT = (84, 104)           # (nobody in view, someone in view)
EYE_CHAMFER = 34                 # Size of the cut corners
SENSOR_WIDTH = 54                # The dark bar inside each eye that does the looking
SMILE_EYE_RISE = 18              # Greeting: the eyes' lower edge rises by this much
BROW_HEIGHT_ABOVE_EYE = (92, 104)
BROW_WIDTH, BROW_THICKNESS = 250, 12
MOUTH_Y = 215
MOUTH_WIDTH, MOUTH_THICKNESS = 170, 16
MOUTH_WIDTH_GREETING = 230
MOUTH_SMILE_LIFT = 26            # Greeting: the ends of the mouth rise by this much
MOUTH_OPEN_SPEAKING = 40         # Speaking: the mouth opens by up to this much

# Status strip along the bottom, like a duty display
STRIP_Y, STRIP_WIDTH, STRIP_HEIGHT = 432, 520, 78
STATUS_ON_DUTY, STATUS_GREETING = 'ON PATROL', 'HOW CAN I HELP?'
STATUS_SPEAKING = "I'M TELE"

SCAN_RANGE = 88                  # How far the sensor bars travel while scanning
SCAN_PERIOD_S = 7.0
GAZE_RANGE_X = 80
CAMERA_WIDTH, CAMERA_HEIGHT = 256, 192
# A person on the camera's left stands on the screen's right for someone facing Tele.
# Set to -1 if the eyes look away from the person instead of at them.
GAZE_MIRROR = 1

# --- Animation Timings ---
BLINK_INTERVAL_MIN = 3.0
BLINK_INTERVAL_MAX = 7.0
BLINK_ANIMATION_SPEED = 0.075
DOUBLE_BLINK_CHANCE = 0.2
NOTICE_DURATION_MS = 350.0       # Patrolling <-> noticing
TRANSITION_DURATION_MS = 700.0   # Noticing <-> greeting
PERSON_TIMEOUT_MS = 1200         # Without news of a person for this long, go back to patrolling


def interpolate(start_val, end_val, phase):
    return start_val + (end_val - start_val) * phase


def mix(pair, phase):
    return interpolate(pair[0], pair[1], phase)


def ease_in_out(t):
    """Smooths out the start and end of the transition (cosine easing)."""
    return (1 - math.cos(t * math.pi)) / 2


class RobotFaceNode(Node):
    def __init__(self):
        super().__init__('robot_face_display')

        self.create_subscription(String, '/person_intent', self.intent_callback, 10)
        # Where the tracked person's eyes are in the camera image: the face looks there
        self.create_subscription(Point2D, '/face_tracker/eye_center', self.eye_center_callback, 10)
        # Tele's mode and speech come from the body
        self.create_subscription(String, '/tele_mode', self.mode_callback, 10)
        self.create_subscription(Bool, '/tele_speaking', self.speaking_callback, 10)
        self.get_logger().info("Robot Face Node Started. Listening to /person_intent")

        # Application State
        self.target_emotion = 'NORMAL'
        self.notice_phase = 0.0   # 0.0 = patrolling, 1.0 = noticing
        self.current_phase = 0.0  # 0.0 = not smiling, 1.0 = greeting
        self.tele_mode = 'PATROL'
        self.last_mode_time = 0
        self.last_speaking_time = -10000
        self.snow = []

        # Gaze
        self.person_gaze = (0.0, 0.0)
        self.last_person_time = -10000
        self.gaze = [0.0, 0.0]

        # Blinking State
        self.eye_state = 'OPEN'
        self.animation_start_time = 0
        self.blinks_left = 0

        # --- Pygame Setup ---
        pygame.init()
        if IS_TESTING_ON_LAPTOP:
            TEST_SIZE = 800
            self.screen = pygame.display.set_mode((TEST_SIZE, TEST_SIZE))
            pygame.mouse.set_visible(True)
        else:
            self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
            pygame.mouse.set_visible(False)

        pygame.display.set_caption("ROS 2 Robot Face")
        self.clock = pygame.time.Clock()

        self.screen_width, self.screen_height = self.screen.get_size()
        self.screen_center_x = self.screen_width // 2
        self.screen_center_y = self.screen_height // 2
        self.scale = min(self.screen_width, self.screen_height) / 1080.0
        self.status_font = pygame.font.Font(None, int(50 * self.scale))
        self.status_cache = {}

        now = pygame.time.get_ticks()
        self.next_blink_time = now + random.uniform(BLINK_INTERVAL_MIN, BLINK_INTERVAL_MAX) * 1000
        self.last_frame_time = now

    # ------------------------------------------------------------------ ROS callbacks
    def intent_callback(self, msg):
        """Triggered when a message is received on /person_intent"""
        intent = msg.data.strip().upper()
        if intent != 'NO_PERSON_DETECTED':
            self.last_person_time = pygame.time.get_ticks()

        # Only smile if the intent is exactly WANT_TO_INTERACT
        if intent == 'WANT_TO_INTERACT':
            if self.target_emotion != 'SMILE':
                self.target_emotion = 'SMILE'
                self.get_logger().info(f"Person Intent: {intent} -> Emotion set to SMILE")
        else:
            if self.target_emotion != 'NORMAL':
                self.target_emotion = 'NORMAL'
                self.get_logger().info(f"Person Intent: {intent} -> Emotion set to NORMAL")

    def eye_center_callback(self, msg):
        x = (CAMERA_WIDTH / 2 - msg.x) / (CAMERA_WIDTH / 2)
        y = (msg.y - CAMERA_HEIGHT / 2) / (CAMERA_HEIGHT / 2)
        self.person_gaze = (max(-1.0, min(1.0, x * GAZE_MIRROR)), max(-1.0, min(1.0, y)))
        self.last_person_time = pygame.time.get_ticks()

    def mode_callback(self, msg):
        self.tele_mode = msg.data.strip().upper()
        self.last_mode_time = pygame.time.get_ticks()

    def speaking_callback(self, msg):
        self.last_speaking_time = pygame.time.get_ticks() if msg.data else -10000

    def christmas_active(self):
        if self.tele_mode != 'CHRISTMAS':
            return False
        if pygame.time.get_ticks() - self.last_mode_time > 5000:
            self.tele_mode = 'PATROL'  # The mode manager went silent
            return False
        return True

    # ------------------------------------------------------------------ drawing helpers
    def px(self, x, y):
        """Design coordinates (relative to the screen centre, 1080 scale) to screen pixels."""
        return (int(self.screen_center_x + x * self.scale), int(self.screen_center_y + y * self.scale))

    def rect_at(self, x, y, w, h):
        rect = pygame.Rect(0, 0, max(1, int(w * self.scale)), max(1, int(h * self.scale)))
        rect.center = self.px(x, y)
        return rect

    def rounded_rect(self, color, x, y, w, h, radius):
        pygame.draw.rect(self.screen, color, self.rect_at(x, y, w, h),
                         border_radius=int(min(radius, w / 2, h / 2) * self.scale))

    def polygon(self, color, points, grow=0.0):
        """Filled polygon from design coordinates. grow enlarges it around its centre (for the glow)."""
        if grow:
            cx = sum(p[0] for p in points) / len(points)
            cy = sum(p[1] for p in points) / len(points)
            points = [(x + math.copysign(grow, x - cx), y + math.copysign(grow, y - cy)) for x, y in points]
        pygame.draw.polygon(self.screen, color, [self.px(x, y) for x, y in points])

    def draw_eyes(self, notice, smile, openness, background):
        w, c = EYE_WIDTH, EYE_CHAMFER
        h = max(10, mix(EYE_HEIGHT, notice) * openness)
        c = min(c, h / 2)
        # Greeting: the lower edge rises a little, like eyes narrowing in a smile
        rise = SMILE_EYE_RISE * smile * openness
        for side in (-1, 1):
            ex = side * EYE_X
            # A slit with two cut corners: top outer and bottom inner. o points outwards.
            o = side
            shape = [(ex + x, EYE_Y + y) for x, y in (
                (-o * w / 2, -h / 2), (o * (w / 2 - c), -h / 2), (o * w / 2, -h / 2 + c),
                (o * w / 2, h / 2 - rise), (-o * (w / 2 - c), h / 2 - rise), (-o * w / 2, h / 2 - c - rise))]
            self.polygon(HALO_COLOR, shape, grow=14)
            self.polygon(FACE_COLOR, shape)
            # The sensor bar: dark, with a bright line in its middle
            if openness > 0.5:
                sx = ex + self.gaze[0]
                half = SENSOR_WIDTH / 2
                top, bottom = EYE_Y - h / 2, EYE_Y + h / 2 - rise
                self.polygon(background, [(sx - half, top), (sx + half, top), (sx + half, bottom), (sx - half, bottom)])
                self.polygon(FACE_COLOR, [(sx - 5, top + 12), (sx + 5, top + 12), (sx + 5, bottom - 12), (sx - 5, bottom - 12)])

    def draw_brows(self, notice, smile):
        """Thin level lines above the eyes, with a small corner at the outer end."""
        y = EYE_Y - mix(BROW_HEIGHT_ABOVE_EYE, notice) - 12 * smile
        t = BROW_THICKNESS
        for side in (-1, 1):
            x0, x1 = side * (EYE_X - BROW_WIDTH / 2), side * (EYE_X + BROW_WIDTH / 2)
            self.polygon(FACE_COLOR, [(x0, y), (x1, y), (x1 + side * 22, y + 26), (x1 + side * 8, y + 26),
                                      (x1 - side * 6, y + t), (x0, y + t)])

    def draw_mouth(self, notice, smile, talk, color):
        """A straight line. Greeting lifts its ends into a small smile; speaking opens it."""
        w = interpolate(MOUTH_WIDTH, MOUTH_WIDTH_GREETING, smile)
        t = MOUTH_THICKNESS + MOUTH_OPEN_SPEAKING * talk
        lift = MOUTH_SMILE_LIFT * smile          # How far the two ends rise
        bend = interpolate(0, 44, smile)         # Where the line bends upwards
        y = MOUTH_Y
        self.polygon(color, [(-w / 2, y - lift), (-w / 2 + bend, y), (w / 2 - bend, y), (w / 2, y - lift),
                             (w / 2, y - lift + t), (w / 2 - bend, y + t), (-w / 2 + bend, y + t), (-w / 2, y - lift + t)])

    def draw_scan_line(self, now, strength):
        """A faint line through the eyes with a brighter part sweeping along it: Tele is scanning."""
        if strength < 0.02:
            return
        fade = lambda color: tuple(int(b + (c - b) * strength) for c, b in zip(color, BACKGROUND_COLOR))
        self.polygon(fade(HALO_COLOR), [(-480, EYE_Y - 2), (480, EYE_Y - 2), (480, EYE_Y + 2), (-480, EYE_Y + 2)])
        x = math.sin(now / 1000.0 * 2 * math.pi / SCAN_PERIOD_S) * 390
        self.polygon(fade(DIM_COLOR), [(x - 90, EYE_Y - 4), (x + 90, EYE_Y - 4), (x + 90, EYE_Y + 4), (x - 90, EYE_Y + 4)])

    def draw_frame(self):
        """Corner marks around the face, like a display's targeting frame."""
        k, length = 470, 70
        for sx in (-1, 1):
            for sy in (-1, 1):
                x, y = sx * k, sy * k - (40 if sy > 0 else 0)
                self.polygon(DIM_COLOR, [(x, y), (x - sx * length, y), (x - sx * length, y - sy * 5), (x, y - sy * 5)])
                self.polygon(DIM_COLOR, [(x, y), (x, y - sy * length), (x - sx * 5, y - sy * length), (x - sx * 5, y)])

    def draw_status(self, text, on_duty, now):
        """The strip at the bottom that says what Tele is doing."""
        if text not in self.status_cache:
            self.status_cache[text] = self.status_font.render(' '.join(text), True, TEXT_COLOR)
        label = self.status_cache[text]
        w, h, c = max(STRIP_WIDTH, label.get_width() / self.scale + 150), STRIP_HEIGHT, 22
        outline = [(-w / 2 + c, STRIP_Y - h / 2), (w / 2, STRIP_Y - h / 2), (w / 2, STRIP_Y + h / 2 - c),
                   (w / 2 - c, STRIP_Y + h / 2), (-w / 2, STRIP_Y + h / 2), (-w / 2, STRIP_Y - h / 2 + c)]
        pygame.draw.polygon(self.screen, DIM_COLOR, [self.px(x, y) for x, y in outline], max(2, int(4 * self.scale)))
        self.screen.blit(label, label.get_rect(center=self.px(18, STRIP_Y + 2)))
        # A small square that blinks slowly while on duty
        if not on_duty or (now // 700) % 2 == 0:
            x = -label.get_width() / 2 / self.scale - 22
            self.polygon(FACE_COLOR, [(x - 11, STRIP_Y - 11), (x + 11, STRIP_Y - 11),
                                      (x + 11, STRIP_Y + 11), (x - 11, STRIP_Y + 11)])

    def draw_snow(self):
        if not self.snow:
            self.snow = [[random.uniform(0, self.screen_width), random.uniform(0, self.screen_height),
                          random.uniform(1.5, 4.5), random.randint(3, 7)] for _ in range(80)]
        for flake in self.snow:
            flake[1] += flake[2] * 2
            if flake[1] > self.screen_height:
                flake[0], flake[1] = random.uniform(0, self.screen_width), -10
            pygame.draw.circle(self.screen, (235, 240, 255), (int(flake[0]), int(flake[1])), flake[3])

    def draw_beard(self):
        """A fluffy white beard behind the mouth."""
        for i in range(13):
            angle = math.pi * i / 12
            pygame.draw.circle(self.screen, (250, 250, 250),
                               self.px(math.cos(angle) * 340, 185 + math.sin(angle) * 280), int(95 * self.scale))
        pygame.draw.ellipse(self.screen, (250, 250, 250), self.rect_at(0, 280, 680, 360))

    def draw_hat_and_cheeks(self):
        for side in (-1, 1):
            pygame.draw.circle(self.screen, (255, 130, 150), self.px(side * 420, 95), int(50 * self.scale))
        brim_y = -375
        tip = self.px(250, brim_y - 135)
        pygame.draw.polygon(self.screen, (200, 30, 40), [self.px(-360, brim_y), self.px(360, brim_y), tip])
        self.rounded_rect((250, 250, 250), 0, brim_y, 800, 96, 48)
        pygame.draw.circle(self.screen, (250, 250, 250), tip, int(48 * self.scale))

    # ------------------------------------------------------------------ animation
    def update_gaze(self, now, dt, notice):
        """The sensor bars scan from side to side; they follow a person once one is in view."""
        # Pauses a little at each side, like someone looking down a corridor
        sweep = max(-1.0, min(1.0, 1.35 * math.sin(now / 1000.0 * 2 * math.pi / SCAN_PERIOD_S)))
        target = interpolate(sweep * SCAN_RANGE, self.person_gaze[0] * GAZE_RANGE_X, notice)
        self.gaze[0] += (target - self.gaze[0]) * min(1.0, dt / 160.0)

    def update_blink(self, now):
        """Returns how open the eyes are, from 0 to 1."""
        duration = BLINK_ANIMATION_SPEED * 1000
        if self.eye_state == 'OPEN':
            if now >= self.next_blink_time:
                self.eye_state, self.animation_start_time = 'CLOSING', now
                if self.blinks_left == 0 and random.random() < DOUBLE_BLINK_CHANCE:
                    self.blinks_left = 1
            return 1.0
        progress = (now - self.animation_start_time) / duration
        if self.eye_state == 'CLOSING':
            if progress >= 1.0:
                self.eye_state, self.animation_start_time = 'OPENING', now
                return 0.0
            return 1.0 - progress
        if progress >= 1.0:
            self.eye_state = 'OPEN'
            if self.blinks_left > 0:
                self.blinks_left -= 1
                self.next_blink_time = now + 120
            else:
                self.next_blink_time = now + random.uniform(BLINK_INTERVAL_MIN, BLINK_INTERVAL_MAX) * 1000
            return 1.0
        return progress

    def run(self):
        """Main Loop combining ROS 2 and Pygame"""
        while rclpy.ok():
            # 1. Process ROS 2 callbacks. Several topics arrive faster than the screen
            #    redraws, so handle a few per frame to keep none of them waiting.
            for _ in range(6):
                rclpy.spin_once(self, timeout_sec=0)

            # 2. Process Pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                    self.get_logger().info("Shutting down display...")
                    pygame.quit()
                    sys.exit()

            # --- Time Management ---
            now = pygame.time.get_ticks()
            dt = now - self.last_frame_time
            self.last_frame_time = now

            christmas = self.christmas_active()
            speaking = now - self.last_speaking_time < 700
            person = now - self.last_person_time < PERSON_TIMEOUT_MS
            smiling = self.target_emotion == 'SMILE' or christmas  # Santa always smiles
            awake = person or smiling or speaking

            # --- Smooth changes between the three looks ---
            step = dt / NOTICE_DURATION_MS
            self.notice_phase = min(1.0, self.notice_phase + step) if awake else max(0.0, self.notice_phase - step)
            step = dt / TRANSITION_DURATION_MS
            self.current_phase = min(1.0, self.current_phase + step) if smiling else max(0.0, self.current_phase - step)
            notice, smile = ease_in_out(self.notice_phase), ease_in_out(self.current_phase)
            talk = (0.25 + 0.75 * abs(math.sin(now / 1000.0 * 9.0))) if speaking else 0.0

            self.update_gaze(now, dt, notice)
            openness = self.update_blink(now)

            # --- Drawing Phase ---
            background = (10, 20, 45) if christmas else BACKGROUND_COLOR
            self.screen.fill(background)
            if christmas:
                self.draw_snow()
                self.draw_beard()
            if not christmas:
                self.draw_frame()
                self.draw_scan_line(now, 1.0)
                status = STATUS_SPEAKING if speaking else STATUS_GREETING if smile > 0.5 else STATUS_ON_DUTY
                self.draw_status(status, status == STATUS_ON_DUTY, now)
            self.draw_eyes(notice, smile, openness, background)
            self.draw_brows(notice, smile)
            self.draw_mouth(notice, smile, talk, (200, 30, 40) if christmas else FACE_COLOR)
            if christmas:
                self.draw_hat_and_cheeks()

            pygame.display.flip()
            self.clock.tick(FPS)


def main(args=None):
    rclpy.init(args=args)

    node = RobotFaceNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
