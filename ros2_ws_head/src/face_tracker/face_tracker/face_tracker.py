#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from vision_msgs.msg import Point2D

class FaceTrackerNode(Node):
    def __init__(self):
        super().__init__('face_tracker_node')
        
        # --- Parameters ---
        self.declare_parameter('pitch_topic', '/servo_command')
        self.declare_parameter('roll_topic', '/servo_command_pan')
        self.declare_parameter('eye_center_topic', '/face_tracker/eye_center')
        self.declare_parameter('dead_zone_percent', 30) 
        self.declare_parameter('timeout_sec', 2.0) # Seconds before returning to home
        # Resolution should match what Tracker is using (usually 640x480 for standard webcams)
        # Ideally this should be dynamic or passed as a parameter too.
        self.declare_parameter('image_width', 256)
        self.declare_parameter('image_height', 192)

        # --- Publishers and Subscribers ---
        pitch_topic = self.get_parameter('pitch_topic').get_parameter_value().string_value
        roll_topic = self.get_parameter('roll_topic').get_parameter_value().string_value
        eye_center_topic = self.get_parameter('eye_center_topic').get_parameter_value().string_value
        
        # Maintain absolute target angles
        self.home_pitch = 55.0
        self.home_roll = 300.0
        self.pitch_angle = self.home_pitch  # Tilt home pose
        self.roll_angle = self.home_roll  # Pan home pose
        self.angle_step = 1.0 # Degrees to move per frame when outside deadzone

        self.pitch_publisher = self.create_publisher(Float64, pitch_topic, 10)
        self.roll_publisher = self.create_publisher(Float64, roll_topic, 10)
        
        self.subscription = self.create_subscription(
            Point2D,
            eye_center_topic,
            self.eye_center_callback,
            10)
            
        self.last_eye_center_time = self.get_clock().now()
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        self.get_logger().info('Face tracker CONTROL node has been started. Waiting for eye center data...')

    def timer_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_eye_center_time).nanoseconds / 1e9
        timeout_sec = self.get_parameter('timeout_sec').get_parameter_value().double_value

        if elapsed > timeout_sec:
            # If no face is seen for the timeout duration and we are not at home, go home
            if self.roll_angle != self.home_roll or self.pitch_angle != self.home_pitch:
                self.roll_angle = self.home_roll
                self.pitch_angle = self.home_pitch
                
                roll_msg = Float64()
                roll_msg.data = self.roll_angle
                self.roll_publisher.publish(roll_msg)

                pitch_msg = Float64()
                pitch_msg.data = self.pitch_angle
                self.pitch_publisher.publish(pitch_msg)

    def eye_center_callback(self, msg):
        self.last_eye_center_time = self.get_clock().now()
        width = self.get_parameter('image_width').get_parameter_value().integer_value
        height = self.get_parameter('image_height').get_parameter_value().integer_value

        center_x = width // 2
        center_y = height // 2
        dead_zone_percent = self.get_parameter('dead_zone_percent').get_parameter_value().integer_value
        dead_zone_x = (width * dead_zone_percent) // 200 # Divided by 200 because percent is split on both sides
        dead_zone_y = (height * dead_zone_percent) // 200

        target_x = msg.x
        target_y = msg.y

        # --- Pitch and Roll Calculation ---
        # Roll (left/right) -> Pan
        if target_x < center_x - dead_zone_x: 
            self.roll_angle += self.angle_step # Move left 
        elif target_x > center_x + dead_zone_x:
            self.roll_angle -= self.angle_step # Move right 

        # Pitch (up/down)
        if target_y < center_y - dead_zone_y:
            self.pitch_angle += self.angle_step # Move up
        elif target_y > center_y + dead_zone_y:
            self.pitch_angle -= self.angle_step # Move down 

        # Clamp angles to prevent rotating beyond safe physical limits
        # Tilt is limited to 40-80 degrees based on mechanical constraints
        self.roll_angle = max(0.0, min(360.0, self.roll_angle))
        self.pitch_angle = max(40.0, min(80.0, self.pitch_angle))

        # --- Publish Commands ---
        roll_msg = Float64()
        roll_msg.data = self.roll_angle
        self.roll_publisher.publish(roll_msg)

        pitch_msg = Float64()
        pitch_msg.data = self.pitch_angle
        self.pitch_publisher.publish(pitch_msg)
          

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