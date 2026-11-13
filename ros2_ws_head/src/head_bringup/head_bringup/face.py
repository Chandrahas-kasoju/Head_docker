import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from vision_msgs.msg import Point2D

import pygame
import random
import sys
import math

# --- Configuration Settings ---
BACKGROUND_COLOR = (20, 20, 20)
FACE_COLOR = (0, 200, 255)
HALO_COLOR = (0, 90, 120)        # Soft glow drawn under the bright parts
VISOR_FILL = (8, 34, 44)
GLINT_COLOR = (235, 250, 255)
IS_TESTING_ON_LAPTOP = False
FPS = 30                         # 30 is smooth enough and halves the load of 60

# --- Geometry (in pixels on a 1080 x 1080 screen; scaled to the real screen) ---
VISOR_Y = -100                   # Centre of the visor, relative to the screen centre
VISOR_WIDTH, VISOR_HEIGHT = 860, 330
EYE_SPACING = 380
EYE_WIDTH, EYE_HEIGHT, EYE_RADIUS = 170, 150, 60
EYE_GROW_SMILING = 1.1           # Eyes get a little bigger when smiling
MOUTH_Y = 190
MOUTH_WIDTH_NORMAL, MOUTH_WIDTH_SMILING = 250, 290
MOUTH_CURVE_NORMAL, MOUTH_LINE_THICKNESS = 42, 30
MOUTH_HEIGHT_SMILING = 105

# --- Gaze: how far the eyes can move inside the visor ---
GAZE_MAX_X, GAZE_MAX_Y = 55, 22
CAMERA_WIDTH, CAMERA_HEIGHT = 256, 192
# A person on the camera's left stands on the screen's right for someone facing Tele.
# Set to -1 if the eyes look away from the person instead of at them.
GAZE_MIRROR = 1

# --- Animation Timings ---
BLINK_INTERVAL_MIN = 3.0
BLINK_INTERVAL_MAX = 7.0
BLINK_ANIMATION_SPEED = 0.075
DOUBLE_BLINK_CHANCE = 0.2
TRANSITION_DURATION_MS = 700.0   # Morph between the normal and the smiling face
GLANCE_INTERVAL_MIN, GLANCE_INTERVAL_MAX = 2.5, 6.0

SPEECH_BARS = (50, 110, 180, 135, 210, 110, 160, 75, 40)


def interpolate(start_val, end_val, phase):
    return start_val + (end_val - start_val) * phase


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
        self.current_phase = 0.0  # 0.0 = Normal, 1.0 = Smiling
        self.tele_mode = 'PATROL'
        self.last_mode_time = 0
        self.last_speaking_time = -10000
        self.snow = []

        # Gaze
        self.person_gaze = (0.0, 0.0)
        self.last_person_time = -10000
        self.glance = (0.0, 0.0)
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

        now = pygame.time.get_ticks()
        self.next_blink_time = now + random.uniform(BLINK_INTERVAL_MIN, BLINK_INTERVAL_MAX) * 1000
        self.next_glance_time = now + random.uniform(GLANCE_INTERVAL_MIN, GLANCE_INTERVAL_MAX) * 1000
        self.last_frame_time = now

    # ------------------------------------------------------------------ ROS callbacks
    def intent_callback(self, msg):
        """Triggered when a message is received on /person_intent"""
        intent = msg.data.strip().upper()

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

    def rounded_rect(self, color, x, y, w, h, radius, outline=0):
        k = self.scale
        rect = pygame.Rect(0, 0, max(1, int(w * k)), max(1, int(h * k)))
        rect.center = self.px(x, y)
        pygame.draw.rect(self.screen, color, rect, int(outline * k),
                         border_radius=int(min(radius, w / 2, h / 2) * k))

    def draw_visor(self, now):
        # The outline breathes slowly, so the face never looks frozen
        pulse = 0.88 + 0.12 * math.sin(now / 1000.0 * 2 * math.pi / 4.0)
        outline = tuple(int(c * pulse) for c in FACE_COLOR)
        self.rounded_rect(HALO_COLOR, 0, VISOR_Y, VISOR_WIDTH + 26, VISOR_HEIGHT + 26, VISOR_HEIGHT)
        self.rounded_rect(outline, 0, VISOR_Y, VISOR_WIDTH, VISOR_HEIGHT, VISOR_HEIGHT)
        self.rounded_rect(VISOR_FILL, 0, VISOR_Y, VISOR_WIDTH - 24, VISOR_HEIGHT - 24, VISOR_HEIGHT)

    def draw_eyes(self, phase, openness):
        grow = interpolate(1.0, EYE_GROW_SMILING, phase)
        w, h = EYE_WIDTH * grow, EYE_HEIGHT * grow
        gx, gy = self.gaze[0] * GAZE_MAX_X, self.gaze[1] * GAZE_MAX_Y
        # Nothing drawn for the eyes may spill over the visor's edge
        k = self.scale
        inside = pygame.Rect(0, 0, int((VISOR_WIDTH - 24) * k), int((VISOR_HEIGHT - 24) * k))
        inside.center = self.px(0, VISOR_Y)
        self.screen.set_clip(inside)
        for side in (-1, 1):
            ex, ey = side * EYE_SPACING / 2 + gx, VISOR_Y + gy
            draw_h = max(14, h * openness)
            self.rounded_rect(HALO_COLOR, ex, ey, w + 26, draw_h + 26, EYE_RADIUS + 12)
            self.rounded_rect(FACE_COLOR, ex, ey, w, draw_h, EYE_RADIUS)
            if openness > 0.6:
                # Sparkle
                pygame.draw.circle(self.screen, GLINT_COLOR, self.px(ex + w * 0.24, ey - h * 0.21), int(16 * self.scale))
                # Smiling: a cheek rises and turns the eye into a happy arch
                if phase > 0.01:
                    cheek = pygame.Rect(0, 0, int(w * 1.5 * k), int(h * 1.0 * k))
                    cheek.center = self.px(ex, ey + h * 1.0 - phase * h * 0.55)
                    pygame.draw.ellipse(self.screen, VISOR_FILL, cheek)
        self.screen.set_clip(None)

    def draw_mouth(self, phase, color):
        """A curved smile line that opens into a wide smile."""
        width = interpolate(MOUTH_WIDTH_NORMAL, MOUTH_WIDTH_SMILING, phase)
        steps = 40
        upper, lower = [], []
        for i in range(steps + 1):
            x = -1.0 + 2.0 * i / steps
            curve = math.sqrt(max(0.0, 1.0 - x * x))
            # Normal: a thin curved line. Smiling: flat top, round bottom.
            taper = MOUTH_LINE_THICKNESS / 2 * (0.45 + 0.55 * curve)
            y_up = interpolate(MOUTH_CURVE_NORMAL * curve - taper, 0.0, phase)
            y_down = interpolate(MOUTH_CURVE_NORMAL * curve + taper, MOUTH_HEIGHT_SMILING * curve, phase)
            upper.append(self.px(x * width / 2, MOUTH_Y + y_up))
            lower.append(self.px(x * width / 2, MOUTH_Y + y_down))
        points = upper + lower[::-1]
        pygame.draw.polygon(self.screen, color, points)
        pygame.draw.aalines(self.screen, color, True, points)

    def draw_speech(self, now, color):
        """Sound-wave bars instead of a mouth while Tele is speaking."""
        t = now / 1000.0
        for i, base in enumerate(SPEECH_BARS):
            level = 0.35 + 0.65 * abs(math.sin(t * (5.0 + 1.3 * i) + i * 1.7))
            x = (i - len(SPEECH_BARS) // 2) * 54
            self.rounded_rect(color, x, MOUTH_Y + 50, 30, max(30, base * level), 15)

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
        k = self.scale
        for i in range(13):
            angle = math.pi * i / 12
            pygame.draw.circle(self.screen, (250, 250, 250),
                               self.px(math.cos(angle) * 330, 170 + math.sin(angle) * 290), int(95 * k))
        beard = pygame.Rect(0, 0, int(660 * k), int(380 * k))
        beard.center = self.px(0, 270)
        pygame.draw.ellipse(self.screen, (250, 250, 250), beard)

    def draw_hat_and_cheeks(self):
        k = self.scale
        for side in (-1, 1):
            pygame.draw.circle(self.screen, (255, 130, 150), self.px(side * 400, 120), int(50 * k))
        brim_y = VISOR_Y - VISOR_HEIGHT / 2 - 20
        tip = self.px(250, brim_y - 175)
        pygame.draw.polygon(self.screen, (200, 30, 40), [self.px(-360, brim_y), self.px(360, brim_y), tip])
        self.rounded_rect((250, 250, 250), 0, brim_y, 820, 100, 50)
        pygame.draw.circle(self.screen, (250, 250, 250), tip, int(55 * k))

    # ------------------------------------------------------------------ animation
    def update_gaze(self, now, dt):
        """Looks at the tracked person; with nobody there, glances around now and then."""
        if now - self.last_person_time < 1000:
            target = self.person_gaze
        else:
            if now >= self.next_glance_time:
                self.glance = random.choice(((0.0, 0.0), (0.0, 0.0),
                                             (random.uniform(-0.7, 0.7), random.uniform(-0.5, 0.5))))
                self.next_glance_time = now + random.uniform(GLANCE_INTERVAL_MIN, GLANCE_INTERVAL_MAX) * 1000
            target = self.glance
        follow = min(1.0, dt / 180.0)  # Eases towards the target in about a fifth of a second
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

            # --- Smooth morph between normal and smiling ---
            phase_step = dt / TRANSITION_DURATION_MS
            if self.target_emotion == 'SMILE' or christmas:  # Santa always smiles
                self.current_phase = min(1.0, self.current_phase + phase_step)
            else:
                self.current_phase = max(0.0, self.current_phase - phase_step)
            phase = ease_in_out(self.current_phase)

            self.update_gaze(now, dt)
            openness = self.update_blink(now)

            # --- Drawing Phase ---
            self.screen.fill((10, 20, 45) if christmas else BACKGROUND_COLOR)
            if christmas:
                self.draw_snow()
                self.draw_beard()
            self.draw_visor(now)
            self.draw_eyes(phase, openness)
            mouth_color = (200, 30, 40) if christmas else FACE_COLOR
            if speaking:
                self.draw_speech(now, mouth_color)
            else:
                self.draw_mouth(phase, mouth_color)
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
