#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import mediapipe as mp

class MediaPipeTrackerNode(Node):
    def __init__(self):
        super().__init__('mediapipe_tracker_node')
        
        # --- Parameters ---
        self.declare_parameter('image_topic', 'hospibot/image_raw')
        self.declare_parameter('pitch_topic', '/servo_command')
        self.declare_parameter('roll_topic', '/stepper_panther')
        self.declare_parameter('dead_zone_percent', 10)
        
        # --- MediaPipe Specific Parameters ---
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence', 0.5)
        self.declare_parameter('visibility_threshold', 0.5)
        
        # --- PERFORMANCE OPTIMIZATION PARAMETERS FOR RASPBERRY PI ---
        self.declare_parameter('model_complexity', 0)
        self.declare_parameter('enable_visualization', True)

        # --- Publishers and Subscribers ---
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        pitch_topic = self.get_parameter('pitch_topic').get_parameter_value().string_value
        roll_topic = self.get_parameter('roll_topic').get_parameter_value().string_value
        
        self.subscription = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.pitch_publisher = self.create_publisher(Int32, pitch_topic, 10)
        self.roll_publisher = self.create_publisher(Int32, roll_topic, 10)
        
        self.bridge = CvBridge()
        
        # --- MediaPipe Model Setup ---
        self.get_logger().info("Loading MediaPipe Pose model...")
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        
        min_detection_confidence = self.get_parameter('min_detection_confidence').get_parameter_value().double_value
        min_tracking_confidence = self.get_parameter('min_tracking_confidence').get_parameter_value().double_value
        model_complexity = self.get_parameter('model_complexity').get_parameter_value().integer_value
        self.enable_visualization = self.get_parameter('enable_visualization').get_parameter_value().bool_value
        
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
            model_complexity=model_complexity)
        
        self.get_logger().info(f'MediaPipe tracker node started. Model Complexity: {model_complexity}, Visualization: {self.enable_visualization}')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        height, width, _ = cv_image.shape
        
        cv_image.flags.writeable = False
        image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image_rgb)
        cv_image.flags.writeable = True

        pitch_cmd = Int32()
        roll_cmd = Int32()
        
        # Default State: No target detected. Command is 2 (Go to Home).
        pitch_cmd.data = 2
        roll_cmd.data = 2
        
        dead_zone_percent = self.get_parameter('dead_zone_percent').get_parameter_value().integer_value
        dead_zone_x = (width * dead_zone_percent) // 200
        dead_zone_y = (height * dead_zone_percent) // 200
        center_x = width // 2
        center_y = height // 2
        
        # Check if a pose was detected at all
        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark
            left_eye = landmarks[self.mp_pose.PoseLandmark.LEFT_EYE.value]
            right_eye = landmarks[self.mp_pose.PoseLandmark.RIGHT_EYE.value]
            
            visibility_threshold = self.get_parameter('visibility_threshold').get_parameter_value().double_value

            # Check if both eyes are clearly visible for tracking
            if left_eye.visibility > visibility_threshold and right_eye.visibility > visibility_threshold:
                # State: Actively tracking. Commands are -1, 0, or 1.
                pitch_cmd.data = 0 # Set to 0 initially for the dead zone case
                roll_cmd.data = 0

                left_eye_x = int(left_eye.x * width)
                left_eye_y = int(left_eye.y * height)
                right_eye_x = int(right_eye.x * width)
                right_eye_y = int(right_eye.y * height)
                
                target_x = (left_eye_x + right_eye_x) // 2
                target_y = (left_eye_y + right_eye_y) // 2

                if target_x < center_x - dead_zone_x: roll_cmd.data = 1
                elif target_x > center_x + dead_zone_x: roll_cmd.data = -1
                
                if target_y < center_y - dead_zone_y: pitch_cmd.data = 1
                elif target_y > center_y + dead_zone_y: pitch_cmd.data = -1

                if self.enable_visualization:
                    cv2.circle(cv_image, (target_x, target_y), 7, (255, 0, 0), -1) # Blue for tracking
            else:
                # State: Target present but obscured. Command is 3 (Hold Position).
                pitch_cmd.data = 2
                roll_cmd.data = 2
                self.get_logger().info('Pose detected, but eyes not visible. Publishing hold command (3).', throttle_duration_sec=2)

            if self.enable_visualization:
                self.mp_drawing.draw_landmarks(
                    cv_image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
        
        else:
            # This 'else' corresponds to 'if results.pose_landmarks:'
            # State: No target detected. The command remains the default of 2.
            self.get_logger().info('No target detected. Publishing go to home command (2).', throttle_duration_sec=2)

        self.pitch_publisher.publish(pitch_cmd)
        self.roll_publisher.publish(roll_cmd)

        if self.enable_visualization:
            cv2.rectangle(cv_image, (center_x - dead_zone_x, center_y - dead_zone_y), 
                          (center_x + dead_zone_x, center_y + dead_zone_y), (0, 0, 255), 2)
            cv2.imshow("MediaPipe Tracker", cv_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    mp_tracker_node = MediaPipeTrackerNode()
    try:
        rclpy.spin(mp_tracker_node)
    except KeyboardInterrupt:
        pass
    finally:
        mp_tracker_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()