import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from vision_msgs.msg import Point2D

import pygame
import random
import sys
import math

# Tele's face: a patrol robot on duty.
#   Patrolling : narrow, serious eyes that scan from side to side, level brows, a straight
#                mouth, a scan line across the eyes and "ON PATROL" on the status strip
#   Noticing   : someone is in view -> the eyes open, turn to the person, the brows lift
#   Greeting   : the person wants to interact -> happy eyes and a wide smile
# In Christmas mode the same face wears a Santa hat and beard.

# --- Configuration Settings ---
BACKGROUND_COLOR = (20, 20, 20)
FACE_COLOR = (0, 200, 255)
HALO_COLOR = (0, 80, 106)        # Soft glow under the eyes
STRIP_COLOR = (0, 60, 82)        # Status strip and scan line
ON_DUTY_COLOR = (60, 230, 140)   # Small light on the status strip while patrolling
GLINT_COLOR = (235, 250, 255)
IS_TESTING_ON_LAPTOP = False
FPS = 30

# --- Geometry (pixels on a 1080 x 1080 screen, relative to the centre; scaled to the real screen) ---
EYE_X, EYE_Y = 240, -95
# Each value is (patrolling, noticing)
EYE_WIDTH = (250, 240)
EYE_HEIGHT = (100, 190)
EYE_RADIUS = (50, 85)
PUPIL_RADIUS = (40, 52)
GLINT_RADIUS = (13, 16)
LID_DROP = (0, 0)                # How far the upper eyelid hangs over the eye
BROW_HEIGHT_ABOVE_EYE = (105, 175)
BROW_WIDTH, BROW_THICKNESS = 200, 28
BROW_ANGLE_NOTICING, BROW_ANGLE_GREETING = 2.0, 3.0
MOUTH_Y = 215
MOUTH_WIDTH = (160, 200)
MOUTH_CURVE = (0, 12)            # Straight while patrolling, barely curved when noticing someone
MOUTH_THICKNESS = (24, 26)
MOUTH_WIDTH_GREETING, MOUTH_HEIGHT_GREETING = 320, 125

# Status strip along the bottom, like a duty display
STRIP_Y, STRIP_WIDTH, STRIP_HEIGHT = 432, 640, 84
STATUS_PATROLLING, STATUS_NOTICING, STATUS_GREETING = 'ON PATROL', 'HELLO THERE', 'HOW CAN I HELP?'
STATUS_SPEAKING = "HELLO, I'M TELE"

SCAN_RANGE = 62                  # How far the pupils travel while scanning
SCAN_PERIOD_S = 7.0
GAZE_RANGE_X, GAZE_RANGE_Y = 46, 20
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
        self.status_font = pygame.font.Font(None, int(58 * self.scale))
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

    def draw_eyes(self, notice, smile, openness, background):
        w, h = mix(EYE_WIDTH, notice), mix(EYE_HEIGHT, notice)
        radius = mix(EYE_RADIUS, notice)
        pupil_r, glint_r = mix(PUPIL_RADIUS, notice), mix(GLINT_RADIUS, notice)
        lid = mix(LID_DROP, notice)
        for side in (-1, 1):
            ex = side * EYE_X
            draw_h = max(16, h * openness)
            self.rounded_rect(HALO_COLOR, ex, EYE_Y, w + 30, draw_h + 30, radius + 14)
            self.rounded_rect(FACE_COLOR, ex, EYE_Y, w, draw_h, radius)
            if openness > 0.55:
                # Everything inside the eye stays inside its outline
                self.screen.set_clip(self.rect_at(ex, EYE_Y, w + 32, draw_h + 32))  # Eye and its glow
                px_, py_ = ex + self.gaze[0], EYE_Y + 6 + self.gaze[1] - 14 * smile
                pygame.draw.circle(self.screen, background, self.px(px_, py_), int(pupil_r * self.scale))
                pygame.draw.circle(self.screen, GLINT_COLOR, self.px(px_ + pupil_r * 0.32, py_ - pupil_r * 0.3),
                                   int(glint_r * self.scale))
                # Greeting: a cheek rises and turns the eye into a happy arch
                if smile > 0.01:
                    cheek = self.rect_at(ex, EYE_Y + h * (1.5 - 0.62 * smile), w * 1.7, h * 1.24)
                    pygame.draw.ellipse(self.screen, background, cheek)
                self.screen.set_clip(None)
                # Patrolling: the upper eyelid hangs low, which gives the watchful look
                if lid > 0.5:
                    pygame.draw.rect(self.screen, background,
                                     self.rect_at(ex, EYE_Y - h / 2 + lid / 2 - 20, w + 60, lid + 40))

    def draw_brows(self, notice, smile):
        height = mix(BROW_HEIGHT_ABOVE_EYE, notice) + 8 * smile
        angle = math.radians(interpolate(0.0, BROW_ANGLE_NOTICING, notice) +
                             (BROW_ANGLE_GREETING - BROW_ANGLE_NOTICING) * smile)
        thickness = max(2, int(BROW_THICKNESS * self.scale))
        for side in (-1, 1):
            cx, cy = side * EYE_X, EYE_Y - height
            dx, dy = math.cos(angle) * BROW_WIDTH / 2, math.sin(angle) * BROW_WIDTH / 2 * side
            a, b = self.px(cx - dx, cy - dy), self.px(cx + dx, cy + dy)
            pygame.draw.line(self.screen, FACE_COLOR, a, b, thickness)
            pygame.draw.circle(self.screen, FACE_COLOR, a, thickness // 2)
            pygame.draw.circle(self.screen, FACE_COLOR, b, thickness // 2)

    def draw_mouth(self, notice, smile, talk, color):
        """A calm curved line that grows into a smile, and opens wide when greeting or speaking."""
        width = interpolate(mix(MOUTH_WIDTH, notice), MOUTH_WIDTH_GREETING, smile)
        curve_depth, thickness = mix(MOUTH_CURVE, notice), mix(MOUTH_THICKNESS, notice)
        opened = max(smile, talk * 0.55)
        steps = 40
        upper, lower = [], []
        for i in range(steps + 1):
            x = -1.0 + 2.0 * i / steps
            curve = math.sqrt(max(0.0, 1.0 - x * x))
            taper = thickness / 2 * min(1.0, 0.25 + 2.5 * curve)  # Rounded ends
            y_up = interpolate(curve_depth * curve - taper, 0.0, opened)
            y_down = interpolate(curve_depth * curve + taper, MOUTH_HEIGHT_GREETING * curve, opened)
            upper.append(self.px(x * width / 2, MOUTH_Y + y_up))
            lower.append(self.px(x * width / 2, MOUTH_Y + y_down))
        points = upper + lower[::-1]
        pygame.draw.polygon(self.screen, color, points)
        pygame.draw.aalines(self.screen, color, True, points)

    def draw_scan_line(self, now, strength):
        """A faint line through the eyes with a brighter part sweeping along it: Tele is scanning."""
        if strength < 0.02:
            return
        fade = lambda color: tuple(int(b + (c - b) * strength) for c, b in zip(color, BACKGROUND_COLOR))
        self.rounded_rect(fade(STRIP_COLOR), 0, EYE_Y, 940, 6, 3)
        sweep = math.sin(now / 1000.0 * 2 * math.pi / SCAN_PERIOD_S)
        self.rounded_rect(fade((0, 120, 160)), sweep * 380, EYE_Y, 180, 10, 5)

    def draw_status(self, text, on_duty):
        """The strip at the bottom that says what Tele is doing."""
        self.rounded_rect(STRIP_COLOR, 0, STRIP_Y, STRIP_WIDTH, STRIP_HEIGHT, STRIP_HEIGHT / 2)
        if text not in self.status_cache:
            self.status_cache[text] = self.status_font.render(text, True, GLINT_COLOR)
        label = self.status_cache[text]
        dot_x = -label.get_width() / 2 / self.scale - 34
        pygame.draw.circle(self.screen, ON_DUTY_COLOR if on_duty else GLINT_COLOR,
                           self.px(dot_x, STRIP_Y), int(15 * self.scale))
        self.screen.blit(label, label.get_rect(center=self.px(14, STRIP_Y + 2)))

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
        """Scans from side to side on patrol; looks at the person once one is noticed."""
        # Pauses a little at each side, like someone looking down a corridor
        sweep = max(-1.0, min(1.0, 1.35 * math.sin(now / 1000.0 * 2 * math.pi / SCAN_PERIOD_S)))
        scan = (sweep * SCAN_RANGE, 0.0)
        person = (self.person_gaze[0] * GAZE_RANGE_X, self.person_gaze[1] * GAZE_RANGE_Y)
        target = (interpolate(scan[0], person[0], notice), interpolate(scan[1], person[1], notice))
        follow = min(1.0, dt / 160.0)
        self.gaze[0] += (target[0] - self.gaze[0]) * follow
        self.gaze[1] += (target[1] - self.gaze[1]) * follow

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
                self.draw_scan_line(now, 1.0 - notice)
                if speaking:
                    status = STATUS_SPEAKING
                elif smile > 0.5:
                    status = STATUS_GREETING
                elif notice > 0.5:
                    status = STATUS_NOTICING
                else:
                    status = STATUS_PATROLLING
                self.draw_status(status, status == STATUS_PATROLLING)
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
