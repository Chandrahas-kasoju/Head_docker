import os

# --- KEY CHANGE 1: Set the video driver to 'dummy' BEFORE importing pygame ---
# This forces Pygame to use a CPU-based software renderer from the start.
os.environ['SDL_VIDEODRIVER'] = 'dummy'

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import random

# --- Configuration Settings (Unchanged) ---
SCREEN_WIDTH = 1080
SCREEN_HEIGHT = 1080
BACKGROUND_COLOR = (0, 0, 0)
DEFAULT_COLOR = (0, 200, 255)
EYE_SIZE = 180
EYE_SPACING = 250
EYE_BORDER_RADIUS = 25
MOUTH_WIDTH = 280
MOUTH_HEIGHT = 40
MOUTH_VERTICAL_OFFSET = 160
MOUTH_BORDER_RADIUS = 10
BLINK_INTERVAL_MIN = 3.0
BLINK_INTERVAL_MAX = 7.0
BLINK_ANIMATION_SPEED = 0.075

class RobotFaceNode(Node):
    def __init__(self):
        super().__init__('robot_face_node')
        self.get_logger().info("Robot Face Node has started using software rendering.")

        pygame.init()

        # --- KEY CHANGE 2: Create two surfaces ---
        # 1. The 'actual_screen' is the visible window on your display.
        #    We must create this one first. It can now use hardware acceleration without conflict.
        self.actual_screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.FULLSCREEN)
        
        # 2. The 'drawing_surface' is our off-screen, software-rendered canvas.
        #    All our drawing commands will go to THIS surface.
        self.drawing_surface = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))

        pygame.display.set_caption("Robot Face")
        pygame.mouse.set_visible(False)
        self.clock = pygame.time.Clock()

        # --- ROS 2 and State Variables (Unchanged) ---
        self.expression_subscriber = self.create_subscription(String, '/robot_face/set_expression', self.expression_callback, 10)
        timer_period = 1.0 / 60.0
        self.timer = self.create_timer(timer_period, self.run_one_frame)
        self.eye_color = DEFAULT_COLOR
        self.mouth_color = DEFAULT_COLOR
        screen_center_x = SCREEN_WIDTH // 2
        screen_center_y = SCREEN_HEIGHT // 2
        self.left_eye_center = (screen_center_x - (EYE_SPACING // 2), screen_center_y)
        self.right_eye_center = (screen_center_x + (EYE_SPACING // 2), screen_center_y)
        self.mouth_rect = pygame.Rect(0, 0, MOUTH_WIDTH, MOUTH_HEIGHT)
        self.mouth_rect.center = (screen_center_x, screen_center_y + MOUTH_VERTICAL_OFFSET)
        self.eye_state = 'OPEN'
        self.animation_start_time = 0
        self.next_blink_trigger_time = pygame.time.get_ticks() + random.uniform(BLINK_INTERVAL_MIN, BLINK_INTERVAL_MAX) * 1000

    def expression_callback(self, msg):
        # ... (This function is unchanged)
        expression = msg.data.lower()
        self.get_logger().info(f'Received expression command: "{expression}"')
        if expression == "happy": self.eye_color, self.mouth_color = (100, 255, 100), (100, 255, 100)
        elif expression == "sad": self.eye_color, self.mouth_color = (100, 100, 255), (100, 100, 255)
        elif expression == "angry": self.eye_color, self.mouth_color = (255, 100, 100), (255, 100, 100)
        else: self.eye_color, self.mouth_color = DEFAULT_COLOR, DEFAULT_COLOR

    def run_one_frame(self):
        # ... (This function is mostly unchanged, except for drawing)
        if not rclpy.ok():
            pygame.quit()
            return
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                self.get_logger().info("Shutdown requested, exiting.")
                pygame.quit()
                self.destroy_node()
                rclpy.shutdown()
                return

        current_time = pygame.time.get_ticks()
        # --- Blinking Logic (Unchanged) ---
        if self.eye_state == 'OPEN':
            current_eye_height = EYE_SIZE
            if current_time >= self.next_blink_trigger_time:
                self.eye_state = 'CLOSING'
                self.animation_start_time = current_time
        elif self.eye_state == 'CLOSING':
            progress = (current_time - self.animation_start_time) / (BLINK_ANIMATION_SPEED * 1000)
            if progress >= 1.0: self.eye_state, self.animation_start_time, current_eye_height = 'OPENING', current_time, 0
            else: current_eye_height = EYE_SIZE * (1.0 - progress)
        elif self.eye_state == 'OPENING':
            progress = (current_time - self.animation_start_time) / (BLINK_ANIMATION_SPEED * 1000)
            if progress >= 1.0:
                self.eye_state = 'OPEN'
                self.next_blink_trigger_time = current_time + random.uniform(BLINK_INTERVAL_MIN, BLINK_INTERVAL_MAX) * 1000
                current_eye_height = EYE_SIZE
            else: current_eye_height = EYE_SIZE * progress

        # --- KEY CHANGE 3: All drawing is now done on the 'drawing_surface' ---
        self.drawing_surface.fill(BACKGROUND_COLOR)
        if current_eye_height > 0:
            left_eye_rect = pygame.Rect(0, 0, EYE_SIZE, current_eye_height); left_eye_rect.center = self.left_eye_center
            right_eye_rect = pygame.Rect(0, 0, EYE_SIZE, current_eye_height); right_eye_rect.center = self.right_eye_center
            pygame.draw.rect(self.drawing_surface, self.eye_color, left_eye_rect, border_radius=EYE_BORDER_RADIUS)
            pygame.draw.rect(self.drawing_surface, self.eye_color, right_eye_rect, border_radius=EYE_BORDER_RADIUS)
        pygame.draw.rect(self.drawing_surface, self.mouth_color, self.mouth_rect, border_radius=MOUTH_BORDER_RADIUS)
        
        # --- KEY CHANGE 4: Copy the finished drawing to the visible screen ---
        self.actual_screen.blit(self.drawing_surface, (0, 0))
        pygame.display.flip()
        
        self.clock.tick(60)

def main(args=None):
    # ... (This function is unchanged)
    rclpy.init(args=args)
    robot_face_node = RobotFaceNode()
    try: rclpy.spin(robot_face_node)
    except KeyboardInterrupt: pass
    finally:
        robot_face_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()