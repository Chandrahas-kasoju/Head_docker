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

    servo = ST3215('/dev/ttyACM0')

    def servo_callback(self, msg):
        command = msg.data
        if command == 1:
            self.servo.MoveTo(1,int(100*(4095 / 360)), 500)  # Move to 100 degrees
        elif command == -1:
            self.servo.MoveTo(1, 0, 500)  # Move to 0 degrees
        elif command == 2:
            self.servo.MoveTo(1, int(48*(4095 / 360)), 500)
        else:
            self.servo.StopServo(1)




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