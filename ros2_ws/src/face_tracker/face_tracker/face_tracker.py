#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import os

class FaceTrackerNode(Node):
    def __init__(self):
        super().__init__('face_tracker_node')
        
        # --- Parameters ---
        self.declare_parameter('image_topic', '/hospibot/image_raw')
        self.declare_parameter('pitch_topic', '/servo_command')
        self.declare_parameter('roll_topic', '/stepper_command')
        self.declare_parameter('haar_cascade_path', os.path.join(
            os.path.expanduser('~'), 'Hospibot', 'src', 'face_tracker', 'haarcascade_frontalface_default.xml'))
        self.declare_parameter('dead_zone_percent', 10) # Percentage of the screen to consider as a dead zone

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

        # --- Face Detection Setup ---
        haar_cascade_path = self.get_parameter('haar_cascade_path').get_parameter_value().string_value
        if not os.path.exists(haar_cascade_path):
            self.get_logger().error(f"Haar cascade file not found at: {haar_cascade_path}")
            rclpy.shutdown()
            return

        self.face_cascade = cv2.CascadeClassifier(haar_cascade_path)
        self.get_logger().info('Face tracker node has been started.')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)

        height, width, _ = cv_image.shape
        center_x = width // 2
        center_y = height // 2

        pitch_cmd = Int32()
        roll_cmd = Int32()
        pitch_cmd.data = 0
        roll_cmd.data = 0
        
        dead_zone_percent = self.get_parameter('dead_zone_percent').get_parameter_value().integer_value
        dead_zone_x = (width * dead_zone_percent) // 200
        dead_zone_y = (height * dead_zone_percent) // 200

        if len(faces) > 0:
            # Find the largest face
            largest_face = max(faces, key=lambda item: item[2] * item[3])
            (x, y, w, h) = largest_face

            # Draw a rectangle around the face for visualization
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

            face_center_x = x + w // 2
            face_center_y = y + h // 2

            # --- Pitch and Roll Calculation ---
            # Roll (left/right)
            if face_center_x < center_x - dead_zone_x:
                roll_cmd.data = -1 # Move left
            elif face_center_x > center_x + dead_zone_x:
                roll_cmd.data = 1 # Move right
            
            # Pitch (up/down)
            if face_center_y < center_y - dead_zone_y:
                pitch_cmd.data = 1 # Move up
            elif face_center_y > center_y + dead_zone_y:
                pitch_cmd.data = -1 # Move down

        self.pitch_publisher.publish(pitch_cmd)
        self.roll_publisher.publish(roll_cmd)

        # --- Visualization (optional) ---
        # Draw dead zone
        cv2.rectangle(cv_image, (center_x - dead_zone_x, center_y - dead_zone_y), 
                      (center_x + dead_zone_x, center_y + dead_zone_y), (0, 0, 255), 2)
        cv2.imshow("Face Tracker", cv_image)
        cv2.waitKey(1)

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
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()