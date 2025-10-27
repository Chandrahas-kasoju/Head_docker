#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import message_filters
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageRectifierNode(Node):
    """
    A ROS 2 node that subscribes to a raw image and camera info,
    undistorts the image, and republishes it.
    """
    def __init__(self):
        # Initialize the node with the name 'image_rectifier'
        super().__init__('image_rectifier')

        # Create a CvBridge object to convert between ROS and OpenCV images
        self.bridge = CvBridge()

        # Create a publisher for the undistorted (rectified) image
        self.image_pub = self.create_publisher(Image, '/hospibot/image_rect', 10)

        # Create subscribers for the raw image and camera info topics
        # These subscribers are for the message_filters
        image_sub = message_filters.Subscriber(self, Image, '/hospibot/image_raw')
        info_sub = message_filters.Subscriber(self, CameraInfo, '/hospibot/camera_info')

        # Synchronize the image and camera info topics using an approximate time synchronizer
        # This ensures that we process an image and its corresponding camera info together
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [image_sub, info_sub], 10, 0.1)
        self.time_synchronizer.registerCallback(self.image_callback)

        self.get_logger().info('Image Rectifier node has started.')

    def image_callback(self, image_msg, camera_info_msg):
        """
        Callback function that is executed when synchronized image and camera info are received.
        """
        try:
            # Convert the ROS Image message to an OpenCV image (in BGR8 format)
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Extract the camera matrix and distortion coefficients from the CameraInfo message
        camera_matrix = np.array(camera_info_msg.k).reshape(3, 3)
        dist_coeffs = np.array(camera_info_msg.d)

        # Undistort the received image using the camera parameters
        undistorted_image = cv2.undistort(cv_image, camera_matrix, dist_coeffs)

        try:
            # Convert the undistorted OpenCV image back to a ROS Image message
            rect_image_msg = self.bridge.cv2_to_imgmsg(undistorted_image, "bgr8")
            
            # Set the header of the rectified image to be the same as the input image
            # This preserves the timestamp and frame_id
            rect_image_msg.header = image_msg.header
            
            # Publish the undistorted image
            self.image_pub.publish(rect_image_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish rectified image: {e}')


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create an instance of the ImageRectifierNode
    image_rectifier_node = ImageRectifierNode()

    # Spin the node so the callback function is called.
    # This keeps the node alive to process messages.
    rclpy.spin(image_rectifier_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_rectifier_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()