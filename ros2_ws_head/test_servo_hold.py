#!/usr/bin/env python3
import time
import sys

# We need to make sure st3215 can be imported.
# Try to import it, assuming it's either installed globally or accessible in your environment.
try:
    from st3215 import ST3215
except ImportError:
    print("Error: Could not import st3215 module. Make sure it is installed or in your PYTHONPATH.")
    sys.exit(1)

# Configuration
SERVO_PORT = '/dev/ttyACM0'
SERVO_ID = 1  # Adjust this if your tilt servo ID is different (e.g., 2 for roll)
TARGET_ANGLE_DEG = 45.0

def main():
    print(f"Connecting to ST3215 servo on {SERVO_PORT}...")
    servo = ST3215(SERVO_PORT)
    
    if not servo.PingServo(SERVO_ID):
        print(f"Error: Servo with ID {SERVO_ID} not found. Please check connection and ID.")
        return

    print(f"Servo {SERVO_ID} successfully pinged!")
    
    # Convert degrees to steps (0-4095 steps = 0-360 degrees)
    target_steps = int((TARGET_ANGLE_DEG / 360.0) * 4095.0)
    
    # Clamp to safe physical limits just in case
    target_steps = max(0, min(4095, target_steps))
    
    print(f"Commanding servo to move to {TARGET_ANGLE_DEG} degrees ({target_steps} steps).")
    print("This will use the servo's internal hardware controller (NO custom PID).")

    # MoveTo(id, position, speed, acceleration)
    # Using safe default speeds based on your code
    servo.MoveTo(SERVO_ID, target_steps, 100, 50)
    
    print("\nHolding position... Observe if the motor sags or holds the weight.")
    print("Press Ctrl+C to stop the script and stop the servo.")
    
    try:
        while True:
            # Read position to monitor how well it's holding
            pos = servo.ReadPosition(SERVO_ID)
            if pos is not None:
                current_deg = (pos / 4095.0) * 360.0
                # Use carriage return to print on the same line
                print(f"Current Angle: {current_deg:.2f}° | Target: {TARGET_ANGLE_DEG}° | Error: {abs(TARGET_ANGLE_DEG - current_deg):.2f}°   ", end='\r')
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user. Stopping servo.")
    finally:
        # It's good practice to stop the servo before exiting
        servo.StopServo(SERVO_ID)
        print("Done.")

if __name__ == "__main__":
    main()
