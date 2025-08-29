#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32 
from sensor_msgs.msg import Joy
from st3215 import ST3215
import time

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

    


    def servo_callback(self, msg):
        if self.servo.PingServo(1) == False:
            self.get_logger().error("Servo not connected")
            return
        command = msg.data
        if command == 1:
            self.servo.MoveTo(1,int(80*(4095 / 360)), 100)  # Move to 100 degrees
        elif command == -1:
            self.servo.MoveTo(1, int(30*(4095 / 360)), 100)  # Move to 0 degrees
        elif command == 3:
            self.servo.MoveTo(1, int(56*(4095 / 360)), 100)
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