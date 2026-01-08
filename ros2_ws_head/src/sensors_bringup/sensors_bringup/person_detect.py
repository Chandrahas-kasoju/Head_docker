#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud2, CameraInfo
import tf2_ros
import numpy as np
from image_geometry import PinholeCameraModel
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from vision_msgs.msg import BoundingBox2D, Pose2D, Point2D
from tf2_ros import TransformException




class PersonDetect(Node):

    def __init__(self):
        super().__init__("person_detect")
        # self.subscription = self.create_subscription(
        #     Image,
        #     '/hospibot/image_raw',  # The topic to subscribe to
        #     self.image_callback,
        #     10)
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/hospibot/camera_info',  # Adjust this to your camera info topic
            self.camera_info_callback,
            10)
        
        # self.det_pub = self.create_publisher(
        #     Image , "/person_detect/bb_image", 10)
        # self.camera_info_pub = self.create_publisher(
        #     CameraInfo, '/person_detect/camera_info', 10)
        
        self.bb_sub = self.create_subscription(
            BoundingBox2D, 
            '/hospibot/pose_bbox',
            self.bbox_callback,
            10)
        
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            '/ti_mmwave/radar_scan_pcl',
            self.point_cloud_callback,
            10)
        self.filtered_cloud_pub = self.create_publisher(
            PointCloud2,
            '/person_detect/filtered_points',  # The topic to publish filtered points
            10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.camera_model = None
        #self.model = YOLO('yolov8n.pt')  # Load the YOLO model
        #self.bridge = CvBridge()
        self.current_image = None
        self.camera_info = None
        self.point_cloud = None
        self.x1, self.y1, self.x2, self.y2 = 0, 0, 0, 0  # Initialize bounding box coordinates
        #self.timer= self.create_timer(0.04, self.detect_person)
        
    def bbox_callback(self, msg):
        """
        Callback function to receive bounding box coordinates.
        """
        self.x1 = int(msg.center.position.x - msg.size_x / 2)
        self.y1 = int(msg.center.position.y - msg.size_y / 2)
        self.x2 = int(msg.center.position.x + msg.size_x / 2)
        self.y2 = int(msg.center.position.y + msg.size_y / 2)
        self.get_logger().info(f'Received bounding box: ({self.x1}, {self.y1}), ({self.x2}, {self.y2})')

    def quaternion_to_rotation_matrix(self, q):
        """
        Converts a quaternion into a 3x3 rotation matrix.
        """
        x, y, z, w = q
        x2, y2, z2 = x*x, y*y, z*z
        xy, xz, yz = x*y, x*z, y*z
        wx, wy, wz = w*x, w*y, w*z

        return np.array([
            [1 - 2*y2 - 2*z2,   2*xy - 2*wz,     2*xz + 2*wy],
            [2*xy + 2*wz,       1 - 2*x2 - 2*z2, 2*yz - 2*wx],
            [2*xz - 2*wy,       2*yz + 2*wx,     1 - 2*x2 - 2*y2]
        ])

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
        if self.camera_model is None:
            self.get_logger().warn("Camera model not yet initialized. Skipping point cloud.", throttle_duration_sec=5)
            return
        if self.x1 is None:
            self.get_logger().warn("Bounding box not yet received. Skipping point cloud.", throttle_duration_sec=5)
            return
        self.point_cloud = msg
        points_inside_bbox = []
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                'thermal_camera_link',  # Target frame
                self.point_cloud.header.frame_id,  # Source frame (from the cloud's header)
                rclpy.time.Time()
            )

            # Read only the x, y, and z fields into a (N, 3) NumPy array
            # This directly fixes the IndexError
            xyz_points = pc2.read_points_numpy(
                self.point_cloud,
                field_names=("x", "y", "z"),
                skip_nans=True
            )

            if xyz_points.size == 0:
                self.get_logger().info("Received empty point cloud.")
                return

            # Get rotation and translation from the transform
            q = transform_stamped.transform.rotation
            t = transform_stamped.transform.translation
            
            rotation_matrix = self.quaternion_to_rotation_matrix([q.x, q.y, q.z, q.w])
            translation_vector = np.array([t.x, t.y, t.z])

            # Apply the transformation: R * p + t
            transformed_points = np.dot(xyz_points, rotation_matrix.T) + translation_vector

            # Create a temporary PointCloud2 message for the transformed points
            # This is needed so we can iterate over it in the next step
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'thermal_camera_link'
            
            # Use a helper function that specifically creates an XYZ cloud
            transformed_cloud_msg = pc2.create_cloud_xyz32(header, transformed_points)
            #self.get_logger().info('Point cloud transformed to camera frame.')

        except (TransformException) as e:
            self.get_logger().error(f'Could not transform point cloud: {e}')
            return
        
        # Now, project the transformed points to the image plane
        for point in pc2.read_points(transformed_cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            # Assuming self.camera_model is initialized and valid
            u, v = self.camera_model.project3dToPixel(point)
            
            # Assuming self.x1, self.x2, self.y1, self.y2 are defined
            if self.x1 < u < self.x2 and self.y1 < v < self.y2:
                # Add the 3D point (which is already in the camera frame) to our list
                points_inside_bbox.append(point)

        # Publish the filtered points if any were found
        if points_inside_bbox:
            # The header is already in the correct camera frame
            filtered_cloud_msg = pc2.create_cloud_xyz32(header, points_inside_bbox)
            self.filtered_cloud_pub.publish(filtered_cloud_msg)
            self.get_logger().info(f"Published {len(points_inside_bbox)} points inside the bounding box.")
            self.x1, self.y1, self.x2, self.y2 = 0, 0, 0, 0

        


    
        
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
