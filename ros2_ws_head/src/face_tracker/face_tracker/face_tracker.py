#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from collections import deque

from std_msgs.msg import Float64, String  # Added String import
from vision_msgs.msg import Point2D

class FaceTrackerNode(Node):
    def __init__(self):
        super().__init__('face_tracker_node')
        
        # --- Parameters ---
        self.declare_parameter('pitch_topic', '/servo_command')
        self.declare_parameter('roll_topic', '/servo_command_pan')
        self.declare_parameter('eye_center_topic', '/face_tracker/eye_center')
        self.declare_parameter('timeout_sec', 2.0) # Seconds before returning to home
        # Resolution should match what Tracker is using
        self.declare_parameter('image_width', 256)
        self.declare_parameter('image_height', 192)
        self.declare_parameter('current_pitch_topic', '/servo_current_angle')
        self.declare_parameter('current_roll_topic', '/servo_current_angle_pan')
        # The pixel error is converted to a real angle with the camera's focal length,
        # so the command is "look there" instead of "move a bit more" on every frame.
        self.declare_parameter('focal_length_px', 261.0)
        self.declare_parameter('gain', 0.9)            # Share of the angle error corrected
        self.declare_parameter('latency_sec', 0.15)    # Age of the image when its result arrives
        self.declare_parameter('start_error_deg', 4.0) # Start moving when the face is this far off centre
        self.declare_parameter('stop_error_deg', 1.5)  # Stop once it is this close
        self.declare_parameter('smoothing', 0.4)       # 1.0 = no smoothing of the target

        # --- Publishers and Subscribers ---
        pitch_topic = self.get_parameter('pitch_topic').get_parameter_value().string_value
        roll_topic = self.get_parameter('roll_topic').get_parameter_value().string_value
        eye_center_topic = self.get_parameter('eye_center_topic').get_parameter_value().string_value
        current_pitch_topic = self.get_parameter('current_pitch_topic').get_parameter_value().string_value
        current_roll_topic = self.get_parameter('current_roll_topic').get_parameter_value().string_value
        
        # Maintain absolute target angles
        self.home_pitch = 50.0
        self.home_roll = 300.0
        self.pitch_angle = self.home_pitch  # Tilt home pose
        self.roll_angle = self.home_roll  # Pan home pose
        
        # Track current actual angles from motors
        self.current_pitch = self.home_pitch
        self.current_roll = self.home_roll
        # Recent motor angles, to find where the head was when the image was taken
        self.pitch_history = deque(maxlen=40)
        self.roll_history = deque(maxlen=40)
        self.pitch_moving = False
        self.roll_moving = False

        # Application State
        self.current_intent = 'NORMAL'  # New variable to track intent

        self.pitch_publisher = self.create_publisher(Float64, pitch_topic, 10)
        self.roll_publisher = self.create_publisher(Float64, roll_topic, 10)
        
        self.pitch_sub = self.create_subscription(Float64, current_pitch_topic, self.current_pitch_callback, 10)
        self.roll_sub = self.create_subscription(Float64, current_roll_topic, self.current_roll_callback, 10)
        
        # Eye Tracking Subscriber
        self.subscription = self.create_subscription(
            Point2D,
            eye_center_topic,
            self.eye_center_callback,
            10)
            
        # Intent Subscriber (NEW)
        self.intent_sub = self.create_subscription(
            String,
            '/person_intent',
            self.intent_callback,
            10)
            
        self.last_eye_center_time = self.get_clock().now()
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        self.get_logger().info('Face tracker CONTROL node has been started. Waiting for intent and eye center data...')

    def intent_callback(self, msg):
        """Triggered when a message is received on /person_intent"""
        intent = msg.data.strip().upper()
        
        if intent == 'WANT_TO_INTERACT':
            if self.current_intent != intent:
                self.current_intent = intent
                self.get_logger().info(f"Intent updated to: {self.current_intent} -> Tracking enabled.")
        else:
            if self.current_intent != 'NORMAL':
                self.current_intent = 'NORMAL'
                self.get_logger().info(f"Intent updated to: {intent} (Set as NORMAL) -> Tracking disabled. Returning home.")

    def timer_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_eye_center_time).nanoseconds / 1e9
        timeout_sec = self.get_parameter('timeout_sec').get_parameter_value().double_value

        # Go home if timeout is reached OR if the robot does not want to interact
        if elapsed > timeout_sec or self.current_intent != 'WANT_TO_INTERACT':
            if self.roll_angle != self.home_roll or self.pitch_angle != self.home_pitch:
                self.roll_angle = self.home_roll
                self.pitch_angle = self.home_pitch
                self.roll_moving = False
                self.pitch_moving = False

                roll_msg = Float64()
                roll_msg.data = self.roll_angle
                self.roll_publisher.publish(roll_msg)

                pitch_msg = Float64()
                pitch_msg.data = self.pitch_angle
                self.pitch_publisher.publish(pitch_msg)

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def current_pitch_callback(self, msg):
        self.current_pitch = msg.data
        self.pitch_history.append((self.now_sec(), msg.data))

    def current_roll_callback(self, msg):
        self.current_roll = msg.data
        self.roll_history.append((self.now_sec(), msg.data))

    @staticmethod
    def angle_at(history, when, fallback):
        """Motor angle at a past moment: the newest sample that is not later than it."""
        for stamp, angle in reversed(history):
            if stamp <= when:
                return angle
        return history[0][1] if history else fallback

    def axis_target(self, error_px, past_angle, previous_target, moving, sign):
        """Returns (target angle, moving) for one axis from its pixel error."""
        focal = self.get_parameter('focal_length_px').value
        error_deg = math.degrees(math.atan2(error_px, focal))
        if moving and abs(error_deg) < self.get_parameter('stop_error_deg').value:
            moving = False
        elif not moving and abs(error_deg) > self.get_parameter('start_error_deg').value:
            moving = True
        if not moving:
            return previous_target, False  # Close enough: hold still
        wanted = past_angle + sign * self.get_parameter('gain').value * error_deg
        k = self.get_parameter('smoothing').value
        return previous_target + k * (wanted - previous_target), True

    def eye_center_callback(self, msg):
        # Always update the timestamp so it doesn't immediately time out when tracking resumes
        self.last_eye_center_time = self.get_clock().now()

        # --- NEW LOGIC: Only move motors if intent is WANT_TO_INTERACT ---
        if self.current_intent != 'WANT_TO_INTERACT':
            return  # Exit the function immediately. Do not process tracking.

        width = self.get_parameter('image_width').get_parameter_value().integer_value
        height = self.get_parameter('image_height').get_parameter_value().integer_value

        error_x = width / 2 - msg.x
        error_y = height / 2 - msg.y

        # The image is a little old: use the motor angles from when it was taken,
        # otherwise the head overshoots while it is still moving.
        when = self.now_sec() - self.get_parameter('latency_sec').value
        past_roll = self.angle_at(self.roll_history, when, self.current_roll)
        past_pitch = self.angle_at(self.pitch_history, when, self.current_pitch)

        # Roll (left/right) -> Pan, Pitch (up/down) -> Tilt
        self.roll_angle, self.roll_moving = self.axis_target(
            error_x, past_roll, self.roll_angle, self.roll_moving, -1.0)
        self.pitch_angle, self.pitch_moving = self.axis_target(
            error_y, past_pitch, self.pitch_angle, self.pitch_moving, 1.0)

        # The actuator node is now responsible for clamping angles to safe physical limits

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
