#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from st3215 import ST3215
import time

class PIDController:
    """A standard generic PID Controller implementation"""
    def __init__(self, kp, ki, kd, output_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.min_out, self.max_out = output_limits

    def compute(self, setpoint, measured_value, dt):
        error = setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        
        # Anti-windup / Output clamping
        if self.min_out is not None:
            output = max(self.min_out, output)
        if self.max_out is not None:
            output = min(self.max_out, output)
            
        return output

class GenericServoController(Node):
    def __init__(self):
        super().__init__('generic_servo_controller')
        
        # Declare parameters for flexibility
        self.declare_parameter('kp', 15.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.5)
        self.declare_parameter('servo_port', '/dev/ttyACM0')
        self.declare_parameter('servo_id', 1)
        self.declare_parameter('use_software_pid', False) # Set to True to use custom PID
        
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        port = self.get_parameter('servo_port').value
        self.sts_id = self.get_parameter('servo_id').value
        self.use_software_pid = self.get_parameter('use_software_pid').value
        
        self.get_logger().info(f"Connecting to servo on {port}")
        self.servo = ST3215(port)
        
        if not self.servo.PingServo(self.sts_id):
            self.get_logger().error(f"Servo {self.sts_id} not connected!")
            
        # Target angle in degrees
        self.target_angle_deg = 55.0  # Default starting target
        
        # Initialize PID controller for calculating speed based on position error
        # Max speed for ST3215 is roughly 3400 steps/s
        self.pid = PIDController(self.kp, self.ki, self.kd, output_limits=(-1000, 1000))
        
        # Generic ROS approach: subscribe to a Float64 for the target angle
        self.subscription = self.create_subscription(
            Float64,
            '/servo_command',
            self.target_callback,
            10
        )
        
        # Control loop timer (e.g., 20 Hz)
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        self.last_time = time.time()
        
    def target_callback(self, msg):
        """Callback to update the target angle from ROS topic"""
        self.target_angle_deg = msg.data
        self.get_logger().info(f"Received new target angle: {self.target_angle_deg} degrees")
        
    def control_loop(self):
        """Main control loop running at fixed frequency"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if self.use_software_pid:
            # === OPTION A: SOFTWARE VELOCITY PID ===
            # Read current position in steps
            current_pos_steps = self.servo.ReadPosition(self.sts_id)
            if current_pos_steps is None:
                return
                
            # Convert step position to angle (0-4095 steps = 0-360 degrees)
            current_angle_deg = (current_pos_steps / 4095.0) * 360.0
            
            # Compute PID control signal (desired velocity)
            speed_cmd = self.pid.compute(self.target_angle_deg, current_angle_deg, dt)
            
            # Use Rotate() to set the speed. The sign dictates direction.
            # Stop if we are close enough to avoid jitter
            if abs(self.target_angle_deg - current_angle_deg) < 0.2:
                self.servo.StopServo(self.sts_id)
            else:
                self.servo.Rotate(self.sts_id, int(speed_cmd))
                
        else:
            # === OPTION B: HARDWARE POSITION CONTROLLER (STANDARD) ===
            # ST3215 already has an internal hardware PID loop optimized for the motor.
            # Usually, standard robotics systems just send the mapped target position directly.
            target_steps = int((self.target_angle_deg / 360.0) * 4095.0)
            
            # Clamp to safe physical limits (e.g., 0 to 4095)
            target_steps = max(0, min(4095, target_steps))
            
            # Send position to the servo's internal controller
            self.servo.MoveTo(self.sts_id, target_steps, 1000, 50) 

def main(args=None):
    rclpy.init(args=args)
    node = GenericServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop servo before shutting down
        node.servo.StopServo(node.sts_id)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
