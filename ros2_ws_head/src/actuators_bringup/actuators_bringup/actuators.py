#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Header
from sensor_msgs.msg import Joy, JointState
from st3215 import ST3215
import time
import math

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        self.subscription = self.create_subscription(
            Int32,
            '/servo_command',
            self.servo_callback,
            10
        )
        self.servo = ST3215('/dev/ttyACM0')
        self.sts_id = 1
        
        # RViz /joint_states Publisher
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz
        
        self.current_pan_rads = 0.0 # Placeholder for Stepper Pan
        self.STEPS_PER_REV = 4096.0
        self.RADS_PER_REV = 2.0 * math.pi
        self.center_degree = 55.0

    


    def servo_callback(self, msg):
        if self.servo.PingServo(1) == False:
            self.get_logger().error("Servo not connected")
            return
        command = msg.data
        if command == 1:
            self.servo.MoveTo(1,int(80*(4095 / 360)), 150)  # Move to 100 degrees
        elif command == -1:
            self.servo.MoveTo(1, int(40*(4095 / 360)), 150)  # Move to 0 degrees
        elif command == 2:
            self.servo.MoveTo(1, int(55*(4095 / 360)), 150)
        else:
            self.HoldPosition(1)

        
    def HoldPosition(self, sts_id):
        """
        Stops the servo's movement and makes it hold its current position.

        :param sts_id: Servo ID

        :return: True if the command was successful, otherwise None.
        """
        # Read the current position of the servo
        current_position = self.servo.ReadPosition(self.sts_id)

        if current_position is not None:
            # Command the servo to move to its current position with a speed of 0
            # This will cause the servo to hold its position with active torque.
            return self.servo.MoveTo(sts_id, current_position, 0)
        else:
            return None

    def timer_callback(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['tele_pan_joint', 'tele_tilt_joint']
        
        tilt_rads = 0.0
        try:
            # Read true physical position from ST3215 Magnetic Encoder
            current_position = self.servo.ReadPosition(self.sts_id)
            if current_position is not None:
                # Convert ST3215 steps to degrees
                current_deg = current_position * (360.0 / self.STEPS_PER_REV)
                # Convert degrees to RViz radians based on our center resting point of 55
                tilt_rads = math.radians(current_deg - self.center_degree)
        except Exception as e:
            pass
            
        msg.position = [self.current_pan_rads, tilt_rads]
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