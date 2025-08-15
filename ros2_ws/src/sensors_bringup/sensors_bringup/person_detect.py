#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from ultralytics import YOLO
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud2, CameraInfo
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_ros
import numpy as np
from image_geometry import PinholeCameraModel
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header



class PersonDetect(Node):

    def __init__(self):
        super().__init__("person_detect")
        self.subscription = self.create_subscription(
            Image,
            '/hospibot/image_raw',  # The topic to subscribe to
            self.image_callback,
            10)
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/hospibot/camera_info',  # Adjust this to your camera info topic
            self.camera_info_callback,
            10)
        
        self.det_pub = self.create_publisher(
            Image , "/person_detect/bb_image", 10)
        self.camera_info_pub = self.create_publisher(
            CameraInfo, '/person_detect/camera_info', 10)
        
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            '/filtered_point_cloud',
            self.point_cloud_callback,
            10)
        self.filtered_cloud_pub = self.create_publisher(
            PointCloud2,
            '/person_detect/filtered_points',  # The topic to publish filtered points
            10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.camera_model = None
        self.model = YOLO('yolov8n.pt')  # Load the YOLO model
        self.bridge = CvBridge()
        self.current_image = None
        self.camera_info = None
        self.point_cloud = None
        self.x1, self.y1, self.x2, self.y2 = 0, 0, 0, 0  # Initialize bounding box coordinates
        self.timer= self.create_timer(0.04, self.detect_person)
        

    def camera_info_callback(self, msg):
        """
        Callback function to store camera info.
        """
        self.camera_info = msg
        if self.camera_model is None:
           self.camera_model = PinholeCameraModel()
           self.camera_model.fromCameraInfo(msg)
           self.get_logger().info('Camera model initialized.')

    def image_callback(self, msg):
        """
        Callback function to process incoming images.
        """
        self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def point_cloud_callback(self, msg):
        """
        Callback function to process incoming point clouds.
        """
        self.point_cloud = msg

    def detect_person(self):
        if self.current_image is None or self.camera_info is None or self.point_cloud is None:
            self.get_logger().warn('Waiting for image or camera info...')
            return
        results = self.model(self.current_image, stream=True, classes=[0])  # Class 0 is for persons
        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy()
            for box in boxes:
                self.x1, self.y1, self.x2, self.y2 = map(int, box[:4])
                cv2.rectangle(self.current_image, (self.x1, self.y1), (self.x2, self.y2), (0, 255, 0), 2)
                names = [result.names[cls.item()] for cls in result.boxes.cls.int()]  # class name of each box
                confs = result.boxes.conf  # confidence score of each box
                for name, conf in zip(names, confs):
                    cv2.putText(self.current_image, f"{name} {conf:.2f}", (self.x1, self.y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        ros_image = self.bridge.cv2_to_imgmsg(self.current_image, encoding="bgr8")
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = "thermal_camera_link"
        self.det_pub.publish(ros_image)
        self.camera_info_pub.publish(self.camera_info)
        points_inside_bbox = []
        try:
            # Get the transform from the point cloud frame to the camera frame
            transform = self.tf_buffer.lookup_transform(
                'thermal_camera_link',      # Target frame (camera)
                'ti_mmwave_0',       # Source frame (LiDAR/Radar)
                rclpy.time.Time()                     # Get the latest available transform
            )

            # Transform the point cloud to the camera's coordinate frame
            transformed_cloud = do_transform_cloud(self.point_cloud, transform)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Could not transform point cloud: {e}')
            return
        
        for point in pc2.read_points(transformed_cloud, field_names=("x", "y", "z"), skip_nans=True):
            u, v = self.camera_model.project3dToPixel(point)
            if self.x1 < u < self.x2 and self.y1 < v < self.y2:
                    # If it is, add the original 3D point (in the camera frame) to our list
                    points_inside_bbox.append(point)

            if points_inside_bbox:
                # Create a new PointCloud2 message for the filtered points
                header = Header()
                header.stamp = self.get_clock().now().to_msg()
                # Publish the points in the camera's frame for easier visualization
                header.frame_id = self.camera_info.header.frame_id

                filtered_cloud_msg = pc2.create_cloud_xyz32(header, points_inside_bbox)
                self.filtered_cloud_pub.publish(filtered_cloud_msg)
                #self.get_logger().info(f"Published {len(points_inside_bbox)} points inside the bounding box.")


    
        
def main(args=None):
    rclpy.init(args=args)

    person_detect = PersonDetect()

    try:
        rclpy.spin(person_detect)
    except KeyboardInterrupt:
        pass

    person_detect.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
