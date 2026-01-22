#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from vision_msgs.msg import Point2D

class FaceTrackerNode(Node):
    def __init__(self):
        super().__init__('face_tracker_node')
        
        # --- Parameters ---
        self.declare_parameter('pitch_topic', '/servo_command')
        self.declare_parameter('roll_topic', '/stepper_panther')
        self.declare_parameter('eye_center_topic', '/face_tracker/eye_center')
        self.declare_parameter('dead_zone_percent', 10) 
        # Resolution should match what Tracker is using (usually 640x480 for standard webcams)
        # Ideally this should be dynamic or passed as a parameter too.
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)

        # --- Publishers and Subscribers ---
        pitch_topic = self.get_parameter('pitch_topic').get_parameter_value().string_value
        roll_topic = self.get_parameter('roll_topic').get_parameter_value().string_value
        eye_center_topic = self.get_parameter('eye_center_topic').get_parameter_value().string_value
        
        self.subscription = self.create_subscription(
            Point2D,
            eye_center_topic,
            self.eye_center_callback,
            10)
            
        self.pitch_publisher = self.create_publisher(Int32, pitch_topic, 10)
        self.roll_publisher = self.create_publisher(Int32, roll_topic, 10)
        
        self.get_logger().info('Face tracker CONTROL node has been started. Waiting for eye center data...')

        # --- Intent Subscription ---
        self.current_intent = "UNKNOWN"
        self.intent_subscription = self.create_subscription(
            String,
            '/person_intent',
            self.intent_callback,
            10)

    def intent_callback(self, msg):
        self.current_intent = msg.data

    def eye_center_callback(self, msg):
        # Only track if the person wants to interact
        #if self.current_intent != "WANT_TO_INTERACT":
        #    return

        width = self.get_parameter('image_width').get_parameter_value().integer_value
        height = self.get_parameter('image_height').get_parameter_value().integer_value
        
        center_x = width // 2
        center_y = height // 2

        pitch_cmd = Int32()
        roll_cmd = Int32()
        
        # Default: No movement
        pitch_cmd.data = 0
        roll_cmd.data = 0
        
        dead_zone_percent = self.get_parameter('dead_zone_percent').get_parameter_value().integer_value
        dead_zone_x = (width * dead_zone_percent) // 200
        dead_zone_y = (height * dead_zone_percent) // 200

        target_x = msg.x
        target_y = msg.y
        
        # --- Pitch and Roll Calculation ---
        # Roll (left/right)
        if target_x < center_x - dead_zone_x:
            roll_cmd.data = 1 # Move left 
        elif target_x > center_x + dead_zone_x:
            roll_cmd.data = -1 # Move right 
        
        # Pitch (up/down)
        if target_y < center_y - dead_zone_y:
            pitch_cmd.data = 1 
        elif target_y > center_y + dead_zone_y:
            pitch_cmd.data = -1 

        self.pitch_publisher.publish(pitch_cmd)
        self.roll_publisher.publish(roll_cmd)

def main(args=None):
    rclpy.init(args=args)
    face_tracker_node = FaceTrackerNode()
    try:
        rclpy.spin(face_tracker_node)
    except KeyboardInterrupt:
        pass
    finally:
        face_tracker_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()