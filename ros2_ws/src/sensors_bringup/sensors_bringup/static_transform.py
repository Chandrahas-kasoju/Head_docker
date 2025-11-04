#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import static_transform_broadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
from scipy.spatial.transform import Rotation as R

class StaticTransformPublisher(Node):
    def __init__(self):
        super().__init__('static_transform_publisher')
        self.broadcaster = static_transform_broadcaster.StaticTransformBroadcaster(self)

        #convert transformation matrix to TransformStamped
        tranformation_matrix = np.array([
            # [ 0.04431464, -0.99828149,  0.0383442 , -0.01208765],
            # [ 0.05798395, -0.03574703, -0.99767731,  0.05698596],
            # [ 0.99733348,  0.04643506,  0.05630019,  0.20893541],
            # [ 0.        ,  0.        ,  0.        ,  1.        ],
             [ 0.12932809,  0.99012921, -0.05402218, -0.24906412],
             [ 0.94291933, -0.1059356 ,  0.31572265, -1.26836635],
             [ 0.30688335, -0.09177037, -0.94731242, -0.23170837],
             [ 0.        ,  0.        ,  0.        ,  1.        ],
        ])


        transform = TransformStamped()
        transform.header.frame_id = 'thermal_camera_link'
        transform.child_frame_id = 'ti_mmwave_0'
        transform.transform.translation.x = tranformation_matrix[0, 3]
        transform.transform.translation.y = tranformation_matrix[1, 3]
        transform.transform.translation.z = tranformation_matrix[2, 3]
        rotation_matrix = tranformation_matrix[:3, :3]
        r = R.from_matrix(rotation_matrix)
        quaternion = r.as_quat()
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        self.broadcaster.sendTransform(transform)

        self.get_logger().info("Static transform published from thermal_camera_link to radar_link")

def main(args=None):
    rclpy.init(args=args)
    static_transform_publisher = StaticTransformPublisher()
    try:
        rclpy.spin(static_transform_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        static_transform_publisher.destroy_node()
        rclpy.shutdown()    
