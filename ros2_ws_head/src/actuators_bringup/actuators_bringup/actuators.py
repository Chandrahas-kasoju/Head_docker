#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Header
from sensor_msgs.msg import JointState
from st3215 import ST3215
import math

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        # Subscriptions for absolute position targets (Float64 in degrees)
        self.subscription_tilt = self.create_subscription(
            Float64,
            '/servo_position',
            self.tilt_callback,
            10
        )
        
        self.subscription_pan = self.create_subscription(
            Float64,
            '/servo_position_pan',
            self.pan_callback,
            10
        )
        
        self.servo = ST3215('/dev/ttyACM0')
        self.tilt_id = 1
        self.pan_id = 2  
        
        # RViz /joint_states Publisher
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz
        
        self.STEPS_PER_REV = 4096.0
        
        # Center offsets for RViz visualization (in degrees)
        self.tilt_center_degree = 55.0
        self.pan_center_degree = 180.0
        
        self.servo_speed = 150

    def tilt_callback(self, msg):
        if not self.servo.PingServo(self.tilt_id):
            self.get_logger().error("Tilt Servo not connected")
            return
            
        target_deg = msg.data
        target_steps = int(target_deg * (self.STEPS_PER_REV / 360.0))
        
        # Ensure steps are within valid ST3215 range [0, 4095]
        target_steps = max(0, min(4095, target_steps))
        self.servo.MoveTo(self.tilt_id, target_steps, self.servo_speed)

    def pan_callback(self, msg):
        if not self.servo.PingServo(self.pan_id):
            self.get_logger().error("Pan Servo not connected")
            return
            
        target_deg = msg.data
        target_steps = int(target_deg * (self.STEPS_PER_REV / 360.0))
        
        # Ensure steps are within valid ST3215 range [0, 4095]
        target_steps = max(0, min(4095, target_steps))
        self.servo.MoveTo(self.pan_id, target_steps, self.servo_speed)

    def hold_position(self, sts_id):
        current_position = self.servo.ReadPosition(sts_id)
        if current_position is not None:
            return self.servo.MoveTo(sts_id, current_position, 0)
        return None

    def timer_callback(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['tele_pan_joint', 'tele_tilt_joint']
        
        tilt_rads = 0.0
        pan_rads = 0.0
        
        try:
            # READ TILT
            tilt_pos = self.servo.ReadPosition(self.tilt_id)
            if tilt_pos is not None:
                tilt_deg = tilt_pos * (360.0 / self.STEPS_PER_REV)
                tilt_rads = math.radians(tilt_deg - self.tilt_center_degree)
                
            # READ PAN
            pan_pos = self.servo.ReadPosition(self.pan_id)
            if pan_pos is not None:
                pan_deg = pan_pos * (360.0 / self.STEPS_PER_REV)
                pan_rads = math.radians(pan_deg - self.pan_center_degree)
                
        except Exception as e:
            self.get_logger().warn(f"Encoder read error: {e}")
            
        msg.position = [pan_rads, tilt_rads]
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()