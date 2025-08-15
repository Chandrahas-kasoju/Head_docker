#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np

class ThermalTrackerNode(Node):
    def __init__(self):
        super().__init__('thermal_tracker_node')
        
        # --- Parameters ---
        self.declare_parameter('image_topic', '/hospibot/image_raw')
        self.declare_parameter('pitch_topic', '/servo_command')
        self.declare_parameter('roll_topic', '/stepper_command')
        self.declare_parameter('dead_zone_percent', 10) # Percentage of the screen to consider as a dead zone
        
        # --- CRITICAL PARAMETER FOR THERMAL TRACKING ---
        # This is the pixel brightness value (0-255) to use as a cutoff. 
        # Pixels brighter than this will be considered "hot".
        # You WILL need to tune this value for your specific camera and environment.
        self.declare_parameter('threshold_value', 200) 

        # --- Publishers and Subscribers ---
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        pitch_topic = self.get_parameter('pitch_topic').get_parameter_value().string_value
        roll_topic = self.get_parameter('roll_topic').get_parameter_value().string_value
        
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10)
        self.pitch_publisher = self.create_publisher(Int32, pitch_topic, 10)
        self.roll_publisher = self.create_publisher(Int32, roll_topic, 10)
        
        self.bridge = CvBridge()
        self.get_logger().info('Thermal tracker node has been started.')

    def image_callback(self, msg):
        try:
            # The Infiray camera might publish in BGR format, even if it looks grayscale
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        # Convert to grayscale for thresholding
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # --- Core Thermal Tracking Logic ---
        threshold_value = self.get_parameter('threshold_value').get_parameter_value().integer_value

        # 1. Threshold the image to get only the hot parts
        # All pixels with a value > threshold_value will be set to 255 (white)
        ret, thresh = cv2.threshold(gray, threshold_value, 255, cv2.THRESH_BINARY)
        
        # Optional: Clean up the image to remove noise
        # Use erode and dilate to remove small, stray hot pixels
        thresh = cv2.erode(thresh, None, iterations=2)
        thresh = cv2.dilate(thresh, None, iterations=2)

        # 2. Find contours (outlines) of the hot blobs
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        height, width = gray.shape
        center_x = width // 2
        center_y = height // 2

        pitch_cmd = Int32()
        roll_cmd = Int32()
        pitch_cmd.data = 0
        roll_cmd.data = 0
        
        dead_zone_percent = self.get_parameter('dead_zone_percent').get_parameter_value().integer_value
        dead_zone_x = (width * dead_zone_percent) // 200
        dead_zone_y = (height * dead_zone_percent) // 200

        if len(contours) > 0:
            # 3. Find the largest contour by area
            largest_contour = max(contours, key=cv2.contourArea)

            # 4. Calculate the center of the largest contour
            M = cv2.moments(largest_contour)
            if M["m00"] > 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # Draw the contour and center for visualization
                cv2.drawContours(cv_image, [largest_contour], -1, (0, 255, 0), 2)
                cv2.circle(cv_image, (cX, cY), 7, (0, 255, 0), -1)

                # --- Pitch and Roll Calculation (same as before) ---
                # Roll (left/right)
                if cX < center_x - dead_zone_x:
                    roll_cmd.data = -1 # Move left
                elif cX > center_x + dead_zone_x:
                    roll_cmd.data = 1 # Move right
                
                # Pitch (up/down)
                if cY < center_y - dead_zone_y:
                    pitch_cmd.data = 1 # Move up
                elif cY > center_y + dead_zone_y:
                    pitch_cmd.data = -1 # Move down

        self.pitch_publisher.publish(pitch_cmd)
        self.roll_publisher.publish(roll_cmd)

        # --- Visualization (optional but recommended for tuning) ---
        cv2.rectangle(cv_image, (center_x - dead_zone_x, center_y - dead_zone_y), 
                      (center_x + dead_zone_x, center_y + dead_zone_y), (0, 0, 255), 2)
        cv2.imshow("Thermal Tracker", cv_image)
        cv2.imshow("Thresholded View", thresh) # Show the binary image
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    thermal_tracker_node = ThermalTrackerNode()
    try:
        rclpy.spin(thermal_tracker_node)
    except KeyboardInterrupt:
        pass
    finally:
        thermal_tracker_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()