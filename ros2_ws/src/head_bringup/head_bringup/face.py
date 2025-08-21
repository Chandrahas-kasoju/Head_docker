import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import random

# --- Configuration Settings ---
# Screen and color settings are now class attributes.
SCREEN_WIDTH = 1080
SCREEN_HEIGHT = 1080
BACKGROUND_COLOR = (0, 0, 0)
DEFAULT_COLOR = (0, 200, 255) # Bright Cyan

# Eye and Mouth geometry
EYE_SIZE = 180
EYE_SPACING = 250
EYE_BORDER_RADIUS = 25
MOUTH_WIDTH = 280
MOUTH_HEIGHT = 40
MOUTH_VERTICAL_OFFSET = 160
MOUTH_BORDER_RADIUS = 10

# Animation Timings
BLINK_INTERVAL_MIN = 1.0
BLINK_INTERVAL_MAX = 3.0
BLINK_ANIMATION_SPEED = 0.075

class RobotFaceNode(Node):
    def __init__(self):
        super().__init__('robot_face_node')
        self.get_logger().info("Robot Face Node has started.")

        # --- Initialize Pygame ---
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.FULLSCREEN)
        pygame.display.set_caption("Robot Face")
        pygame.mouse.set_visible(False)
        self.clock = pygame.time.Clock()

        # --- ROS 2 Subscription ---
        # The node listens on this topic for commands to change its expression (color).
        self.expression_subscriber = self.create_subscription(
            String,
            '/robot_face/set_expression',
            self.expression_callback,
            10)
        
        # --- ROS 2 Timer ---
        # This timer will call the run_one_frame method 60 times per second.
        # This replaces the `while running:` loop from the original script.
        timer_period = 1.0 / 60.0  # 60 Hz
        self.timer = self.create_timer(timer_period, self.run_one_frame)

        # --- State and Positioning Variables ---
        # Convert all local variables from the old script into class attributes.
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
        """This function is called every time a message is published to the topic."""
        expression = msg.data.lower()
        self.get_logger().info(f'Received expression command: "{expression}"')
        
        # Change color based on the received message
        if expression == "happy":
            self.eye_color = (100, 255, 100) # Green
            self.mouth_color = (100, 255, 100)
        elif expression == "sad":
            self.eye_color = (100, 100, 255) # Blue
            self.mouth_color = (100, 100, 255)
        elif expression == "angry":
            self.eye_color = (255, 100, 100) # Red
            self.mouth_color = (255, 100, 100)
        else: # "default" or any other string
            self.eye_color = DEFAULT_COLOR
            self.mouth_color = DEFAULT_COLOR

    def run_one_frame(self):
        """This method contains all the logic and drawing for a single frame."""
        if not rclpy.ok(): # Exit if ROS is shutting down
            pygame.quit()
            return

        # Check for Pygame events (like closing the window or pressing ESC)
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                # If the window is closed, initiate ROS shutdown and exit.
                self.get_logger().info("Shutdown requested, exiting.")
                pygame.quit()
                self.destroy_node()
                rclpy.shutdown()
                return

        current_time = pygame.time.get_ticks()

        # --- Blinking Animation Logic ---
        # (This is the same logic as the previous script, using self. attributes)
        if self.eye_state == 'OPEN':
            current_eye_height = EYE_SIZE
            if current_time >= self.next_blink_trigger_time:
                self.eye_state = 'CLOSING'
                self.animation_start_time = current_time
        elif self.eye_state == 'CLOSING':
            progress = (current_time - self.animation_start_time) / (BLINK_ANIMATION_SPEED * 1000)
            if progress >= 1.0:
                self.eye_state = 'OPENING'
                self.animation_start_time = current_time
                current_eye_height = 0
            else:
                current_eye_height = EYE_SIZE * (1.0 - progress)
        elif self.eye_state == 'OPENING':
            progress = (current_time - self.animation_start_time) / (BLINK_ANIMATION_SPEED * 1000)
            if progress >= 1.0:
                self.eye_state = 'OPEN'
                self.next_blink_trigger_time = current_time + random.uniform(BLINK_INTERVAL_MIN, BLINK_INTERVAL_MAX) * 1000
                current_eye_height = EYE_SIZE
            else:
                current_eye_height = EYE_SIZE * progress

        # --- Drawing Frame by Frame ---
        self.screen.fill(BACKGROUND_COLOR)

        if current_eye_height > 0:
            left_eye_rect = pygame.Rect(0, 0, EYE_SIZE, current_eye_height)
            right_eye_rect = pygame.Rect(0, 0, EYE_SIZE, current_eye_height)
            left_eye_rect.center = self.left_eye_center
            right_eye_rect.center = self.right_eye_center
            pygame.draw.rect(self.screen, self.eye_color, left_eye_rect, border_radius=EYE_BORDER_RADIUS)
            pygame.draw.rect(self.screen, self.eye_color, right_eye_rect, border_radius=EYE_BORDER_RADIUS)

        pygame.draw.rect(self.screen, self.mouth_color, self.mouth_rect, border_radius=MOUTH_BORDER_RADIUS)
        pygame.display.flip()
        self.clock.tick(60)

def main(args=None):
    rclpy.init(args=args)
    robot_face_node = RobotFaceNode()
    try:
        rclpy.spin(robot_face_node)
    except KeyboardInterrupt:
        pass
    finally:
        # This will be called upon shutdown (e.g., Ctrl+C)
        robot_face_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()