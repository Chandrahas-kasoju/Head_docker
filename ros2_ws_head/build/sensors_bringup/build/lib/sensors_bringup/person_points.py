#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

class DynamicPointCloudFilter(Node):
    """
    A ROS2 node that subscribes to a PointCloud2 topic, filters points based on
    spatial constraints, and republishes the filtered cloud with all original
    data fields preserved.
    """
    def __init__(self):
        super().__init__('dynamic_point_cloud_filter')

        # Declare parameters for filter boundaries
        self.declare_parameter('x_range', 3.0)
        self.declare_parameter('y_range', 2.0)
        self.declare_parameter('z_range', 2.0)

        # Create subscription to the input point cloud topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/ti_mmwave/radar_scan_pcl',  # The topic to subscribe to
            self.listener_callback,
            10)

        # Create publisher for the filtered point cloud topic
        self.publisher = self.create_publisher(
            PointCloud2,
            '/filtered_point_cloud',  # The topic to publish to
            10)
        self.get_logger().info('Dynamic Point Cloud Filter node has been started.')

    def listener_callback(self, msg: PointCloud2):
        """
        Callback function to filter the received PointCloud2 message.
        """
        # Get the filter range values from parameters
        x_range = self.get_parameter('x_range').get_parameter_value().double_value
        y_range = self.get_parameter('y_range').get_parameter_value().double_value
        z_range = self.get_parameter('z_range').get_parameter_value().double_value

        # Get the field names from the message itself
        field_names = [field.name for field in msg.fields]

        # Use the point_cloud2 generator to read points and their fields
        # This reads all fields specified in 'field_names' for each point
        points_generator = point_cloud2.read_points(
            msg,
            field_names=field_names,
            skip_nans=True
        )

        filtered_points = []
        for point in points_generator:
            # The generator returns a tuple, so we can map it to a dictionary
            # for easy access to fields by name.
            point_data = dict(zip(field_names, point))

            # Apply the filter conditions using the x, y, and z values
            if (abs(point_data['x']) < x_range and
                abs(point_data['y']) < y_range and
                abs(point_data['z']) < z_range):
                # If the point is within the desired range, add its data
                # (in the original tuple format) to our list.
                filtered_points.append(point)

        # If there are any points left after filtering, create a new cloud
        if filtered_points:
            # Create a new PointCloud2 message.
            # CRITICAL: We use the header and fields from the ORIGINAL message.
            # This preserves the entire data structure (including all fields
            # like intensity, velocity, etc.).
            filtered_cloud_msg = point_cloud2.create_cloud(
                msg.header,  # Use original header
                msg.fields,  # Use original fields structure
                filtered_points
            )

            self.publisher.publish(filtered_cloud_msg)
            self.get_logger().info(f'Published {len(filtered_points)} filtered points.')

def main(args=None):
    rclpy.init(args=args)
    dynamic_point_cloud_filter = DynamicPointCloudFilter()
    rclpy.spin(dynamic_point_cloud_filter)
    dynamic_point_cloud_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()