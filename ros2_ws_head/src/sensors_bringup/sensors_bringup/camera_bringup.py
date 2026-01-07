#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud2, CameraInfo
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import image_geometry
import numpy as np



class Camera(Node):

    def __init__(self):
        super().__init__("camera_node")
        self.det_pub = self.create_publisher(
            Image , "/hospibot/image_raw", 10)
        self.pub = self.create_publisher(CameraInfo, '/hospibot/camera_info', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture('/dev/video4')  # Open the thermalcamera
        
        #

        # Distortion Coefficients (D) in the format: [k1, k2, p1, p2, k3]
        # k1, k2, k3: radial distortion coefficients
        # p1, p2: tangential distortion coefficients
        #dist_coeffs = np.array([-0.22447367, 0.01457183, -0.00027492, -0.01633117, 0.07505037], dtype=np.float32)
        self.msg = CameraInfo()
        self.msg.header.frame_id = "thermal_camera_link"
        self.msg.width = 256
        self.msg.height = 192
        self.msg.k = [262.45607046, 0.0, 98.71235158,
                 0.0, 260.0900099, 120.74086798,
                 0.0, 0.0, 1.0]  #[267.0, 0.0, 128.0, 0.0, 267.0, 99.0, 0.0, 0.0, 1.0]
        self.msg.p = [262.45607046, 0.0, 98.71235158, 0.0,
                 0.0, 260.0900099, 120.74086798, 0.0,
                 0.0, 0.0, 1.0, 0.0] # [267.0, 0.0, 128.0, 0.0, 0.0, 267.0, 99.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.msg.distortion_model = "plumb_bob"
        #self.msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.msg.d = [-0.22447367, 0.01457183, -0.00027492, -0.01633117, 0.07505037]
        self.timer= self.create_timer(0.04, self.publish_image)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open video capture")
            return
        example_param = self.declare_parameter("example_param", "default_value").value
        self.get_logger().info(f"Declared parameter 'example_param'. Value: {example_param}")
        self.get_logger().info("Hello world from the Python node person_detect")

    def publish_image(self):

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to read frame from video capture")
            return
        height = frame.shape[0]//2
        top_half = frame[:height, :] 
        top_flipped = cv2.flip(top_half, 0)
        top_flipped = cv2.flip(top_flipped, 1)
        ros_image = self.bridge.cv2_to_imgmsg(top_flipped, encoding="bgr8")
        stamp = self.get_clock().now().to_msg()
        ros_image.header.stamp = stamp
        self.msg.header.stamp = stamp
        ros_image.header.frame_id = "thermal_camera_link"
        self.msg.header.frame_id = "thermal_camera_link"
        self.det_pub.publish(ros_image)
        self.pub.publish(self.msg)

    
        
def main(args=None):
    rclpy.init(args=args)

    camera = Camera()

    try:
        rclpy.spin(camera)
    except KeyboardInterrupt:
        pass

    camera.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
