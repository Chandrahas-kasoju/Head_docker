import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import pygame
import random
import sys
import math

# --- Configuration Settings ---
SCREEN_WIDTH = 1080
SCREEN_HEIGHT = 1080
BACKGROUND_COLOR = (20, 20, 20)
FACE_COLOR = (0, 200, 255)
IS_TESTING_ON_LAPTOP = False

# --- Geometry Parameters ---
EYE_SPACING = 400

# Normal State (Rounded Squares + Pill/Capsule Mouth)
EYE_WIDTH_NORMAL, EYE_HEIGHT_NORMAL, EYE_BORDER_RADIUS_NORMAL = 180, 180, 25
MOUTH_WIDTH_NORMAL, MOUTH_HEIGHT_NORMAL, MOUTH_Y_OFFSET_NORMAL = 280, 70, 250

# Smiling State (Rounded Squares + Flat Top/Half-Ellipse Mouth)
EYE_WIDTH_SMILING, EYE_HEIGHT_SMILING, EYE_BORDER_RADIUS_SMILING = 230, 230, 35
MOUTH_WIDTH_SMILING, MOUTH_HEIGHT_SMILING, MOUTH_Y_OFFSET_SMILING = 350, 140, 200

# --- Animation Timings ---
BLINK_INTERVAL_MIN = 3.0
BLINK_INTERVAL_MAX = 7.0
BLINK_ANIMATION_SPEED = 0.075

# How long the morph takes (in milliseconds)
TRANSITION_DURATION_MS = 700.0  

def interpolate(start_val, end_val, phase):
    return start_val + (end_val - start_val) * phase

def ease_in_out(t):
    """Smooths out the start and end of the transition (cosine easing)."""
    return (1 - math.cos(t * math.pi)) / 2

class RobotFaceNode(Node):
    def __init__(self):
        super().__init__('robot_face_display')
        
        # ROS 2 Subscriber: Listens to the Person Intent Classifier
        self.subscription = self.create_subscription(
            String,
            '/person_intent',
            self.intent_callback,
            10
        )
        self.get_logger().info("Robot Face Node Started. Listening to /person_intent")

        # Application State
        self.target_emotion = 'NORMAL'
        self.current_phase = 0.0  # 0.0 = Normal, 1.0 = Smiling
        
        # Time tracking
        self.last_frame_time = 0
        
        # Blinking State
        self.eye_state = 'OPEN'
        self.animation_start_time = 0
        
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
        
        self.left_eye_center = (self.screen_center_x - (EYE_SPACING // 2), self.screen_center_y - 100)
        self.right_eye_center = (self.screen_center_x + (EYE_SPACING // 2), self.screen_center_y - 100)
        
        self.next_blink_time = pygame.time.get_ticks() + random.uniform(BLINK_INTERVAL_MIN, BLINK_INTERVAL_MAX) * 1000
        self.last_frame_time = pygame.time.get_ticks()

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

    def run(self):
        """Main Loop combining ROS 2 and Pygame"""
        while rclpy.ok():
            # 1. Process ROS 2 callbacks (non-blocking)
            rclpy.spin_once(self, timeout_sec=0.01)

            # 2. Process Pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                    self.get_logger().info("Shutting down display...")
                    pygame.quit()
                    sys.exit()

            # --- Time Management ---
            current_time = pygame.time.get_ticks()
            dt = current_time - self.last_frame_time
            self.last_frame_time = current_time

            # --- Update Animation Phase (Smooth Time-Based Morphing) ---
            phase_step = dt / TRANSITION_DURATION_MS 
            
            if self.target_emotion == 'SMILE':
                self.current_phase = min(1.0, self.current_phase + phase_step)
            else:
                self.current_phase = max(0.0, self.current_phase - phase_step)

            eased_phase = ease_in_out(self.current_phase)

            # --- Calculate Current Geometry ---
            current_eye_width = int(interpolate(EYE_WIDTH_NORMAL, EYE_WIDTH_SMILING, eased_phase))
            current_eye_base_height = int(interpolate(EYE_HEIGHT_NORMAL, EYE_HEIGHT_SMILING, eased_phase))
            current_radius = int(interpolate(EYE_BORDER_RADIUS_NORMAL, EYE_BORDER_RADIUS_SMILING, eased_phase))
            
            current_m_width = int(interpolate(MOUTH_WIDTH_NORMAL, MOUTH_WIDTH_SMILING, eased_phase))
            current_m_y = int(interpolate(MOUTH_Y_OFFSET_NORMAL, MOUTH_Y_OFFSET_SMILING, eased_phase))

            # --- Blinking Logic ---
            if self.eye_state == 'OPEN':
                current_eye_draw_height = current_eye_base_height
                if current_time >= self.next_blink_time:
                    self.eye_state, self.animation_start_time = 'CLOSING', current_time
            elif self.eye_state == 'CLOSING':
                progress = (current_time - self.animation_start_time) / (BLINK_ANIMATION_SPEED * 1000)
                if progress >= 1.0:
                    self.eye_state, self.animation_start_time, current_eye_draw_height = 'OPENING', current_time, 0
                else:
                    current_eye_draw_height = current_eye_base_height * (1.0 - progress)
            elif self.eye_state == 'OPENING':
                progress = (current_time - self.animation_start_time) / (BLINK_ANIMATION_SPEED * 1000)
                if progress >= 1.0:
                    self.eye_state = 'OPEN'
                    self.next_blink_time = current_time + random.uniform(BLINK_INTERVAL_MIN, BLINK_INTERVAL_MAX) * 1000
                    current_eye_draw_height = current_eye_base_height
                else:
                    current_eye_draw_height = current_eye_base_height * progress

            # --- Drawing Phase ---
            self.screen.fill(BACKGROUND_COLOR)

            # Draw Eyes
            if current_eye_draw_height > 0:
                left_eye_rect = pygame.Rect(0, 0, current_eye_width, current_eye_draw_height)
                left_eye_rect.center = self.left_eye_center
                right_eye_rect = pygame.Rect(0, 0, current_eye_width, current_eye_draw_height)
                right_eye_rect.center = self.right_eye_center
                
                pygame.draw.rect(self.screen, FACE_COLOR, left_eye_rect, border_radius=current_radius)
                pygame.draw.rect(self.screen, FACE_COLOR, right_eye_rect, border_radius=current_radius)

            # --- Draw Morphing Polygon Mouth ---
            steps = 40
            upper_points = []
            lower_points = []
            
            for i in range(steps + 1):
                t = i / steps
                x = -1.0 + 2.0 * t  # Interpolates from -1.0 to 1.0
                
                # ----------------------------------------------------
                # NORMAL SHAPE: Pill/Capsule Shape
                # ----------------------------------------------------
                R = MOUTH_HEIGHT_NORMAL / 2.0
                # Calculate where the straight flat edge turns into a circle
                x_cut = 1.0 - (MOUTH_HEIGHT_NORMAL / MOUTH_WIDTH_NORMAL)
                
                if abs(x) <= x_cut:
                    y_up_norm = -R
                    y_down_norm = R
                else:
                    # Calculate the curved circular caps of the pill shape
                    dx = (abs(x) - x_cut) * (MOUTH_WIDTH_NORMAL / 2.0)
                    y_mag = math.sqrt(max(0.0, R*R - dx*dx))
                    y_up_norm = -y_mag
                    y_down_norm = y_mag
                
                # ----------------------------------------------------
                # SMILING SHAPE: Flat top, half-ellipse bottom
                # ----------------------------------------------------
                y_up_smile = 0.0
                y_down_smile = (MOUTH_HEIGHT_SMILING / 2.0) * math.sqrt(max(0.0, 1.0 - x*x))
                
                # --- Interpolate based on the current animation phase ---
                curr_y_up = interpolate(y_up_norm, y_up_smile, eased_phase)
                curr_y_down = interpolate(y_down_norm, y_down_smile, eased_phase)
                
                curr_x = x * (current_m_width / 2.0)
                
                upper_points.append((curr_x, curr_y_up))
                lower_points.append((curr_x, curr_y_down))
                
            # Combine into a single closed polygon (upper goes left->right, lower goes right->left)
            mouth_points = upper_points + lower_points[::-1]
            
            # Translate local coordinates to screen coordinates
            offset_points = [(self.screen_center_x + px, self.screen_center_y + current_m_y + py) for px, py in mouth_points]
            
            # Draw and smooth the polygon
            pygame.draw.polygon(self.screen, FACE_COLOR, offset_points)
            pygame.draw.aalines(self.screen, FACE_COLOR, True, offset_points)

            pygame.display.flip()
            self.clock.tick(60)

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
