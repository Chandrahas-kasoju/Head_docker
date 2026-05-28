#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from vision_msgs.msg import Point2D
from rcl_interfaces.msg import SetParametersResult

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
        self.declare_parameter('current_pitch_topic', '/servo_current_angle')
        self.declare_parameter('current_roll_topic', '/servo_current_angle_pan')
        self.declare_parameter('Kp_pan', 0.07)
        self.declare_parameter('Kp_tilt', 0.07)

        # --- Publishers and Subscribers ---
        pitch_topic = self.get_parameter('pitch_topic').get_parameter_value().string_value
        roll_topic = self.get_parameter('roll_topic').get_parameter_value().string_value
        eye_center_topic = self.get_parameter('eye_center_topic').get_parameter_value().string_value
        current_pitch_topic = self.get_parameter('current_pitch_topic').get_parameter_value().string_value
        current_roll_topic = self.get_parameter('current_roll_topic').get_parameter_value().string_value
        
        # Maintain absolute target angles
        self.home_pitch = 63.0
        self.home_roll = 300.0
        self.pitch_angle = self.home_pitch  # Tilt home pose
        self.roll_angle = self.home_roll  # Pan home pose
        
        # Track current actual angles from motors
        self.current_pitch = self.home_pitch
        self.current_roll = self.home_roll
        self.Kp_pan = self.get_parameter('Kp_pan').get_parameter_value().double_value
        self.Kp_tilt = self.get_parameter('Kp_tilt').get_parameter_value().double_value
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.pitch_publisher = self.create_publisher(Float64, pitch_topic, 10)
        self.roll_publisher = self.create_publisher(Float64, roll_topic, 10)
        
        self.pitch_sub = self.create_subscription(Float64, current_pitch_topic, self.current_pitch_callback, 10)
        self.roll_sub = self.create_subscription(Float64, current_roll_topic, self.current_roll_callback, 10)
        
        self.subscription = self.create_subscription(
            Point2D,
            eye_center_topic,
            self.eye_center_callback,
            10)
            
        self.last_eye_center_time = self.get_clock().now()
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        self.get_logger().info('Face tracker CONTROL node has been started. Waiting for eye center data...')

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'Kp_pan':
                self.Kp_pan = param.value
                self.get_logger().info(f"Updated Kp_pan to {self.Kp_pan}")
            elif param.name == 'Kp_tilt':
                self.Kp_tilt = param.value
                self.get_logger().info(f"Updated Kp_tilt to {self.Kp_tilt}")
        return SetParametersResult(successful=True)

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

    def current_pitch_callback(self, msg):
        self.current_pitch = msg.data

    def current_roll_callback(self, msg):
        self.current_roll = msg.data

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
        error_x = center_x - target_x
        error_y = center_y - target_y

        # Roll (left/right) -> Pan
        if target_x < center_x - dead_zone_x or target_x > center_x + dead_zone_x:
            self.roll_angle = self.current_roll - (error_x * self.Kp_pan)
        else:
            self.roll_angle = self.current_roll

        # Pitch (up/down)
        if target_y < center_y - dead_zone_y or target_y > center_y + dead_zone_y:
            self.pitch_angle = self.current_pitch + (error_y * self.Kp_tilt)
        else:
            self.pitch_angle = self.current_pitch

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