#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from vision_msgs.msg import BoundingBox2D
from cv_bridge import CvBridge
import cv2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from image_geometry import PinholeCameraModel


class ThermalViz(Node):
    """Simple visualizer that shows visual image and thermal image side-by-side
    and overlays bounding boxes and projected radar points to verify alignment.
    """
    def __init__(self):
        super().__init__('thermal_viz')

        # Topics (can be adjusted via parameters)
        self.declare_parameter('visual_image_topic', '/hospibot/image_raw')
        self.declare_parameter('thermal_image_topic', '/hospibot/thermal_image')
        self.declare_parameter('camera_info_topic', '/hospibot/camera_info')
        # Use the canonical person bounding box topic produced by tracker
        self.declare_parameter('bbox_topic', '/person_bounding_box')
        self.declare_parameter('radar_topic', '/person_detect/filtered_points')
        self.declare_parameter('display_window', 'thermal_viz')
        # Control whether we publish composed visualization to a topic (default: False)
        self.declare_parameter('publish_visualization', False)
        # Default topic for viz (use a dedicated name to avoid conflicts)
        self.declare_parameter('publish_topic', '/thermal_viz/image_raw')
        # If True show the debug window (default: False)
        self.declare_parameter('show_window', False)

        # Publisher for numeric max temperature inside the last bbox
        from std_msgs.msg import Float32  # local import to avoid circular top-level import issues
        self.temp_pub = self.create_publisher(Float32, '/person_max_temperature', 10)

        self.visual_topic = self.get_parameter('visual_image_topic').value
        self.thermal_topic = self.get_parameter('thermal_image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.bbox_topic = self.get_parameter('bbox_topic').value
        self.radar_topic = self.get_parameter('radar_topic').value
        self.window_name = self.get_parameter('display_window').value

        self.publish_viz = self.get_parameter('publish_visualization').value
        self.publish_topic = self.get_parameter('publish_topic').value
        self.show_window = self.get_parameter('show_window').value

        if self.publish_viz:
            self.image_pub = self.create_publisher(Image, self.publish_topic, 10)
        else:
            self.image_pub = None

        self.bridge = CvBridge()

        # Subscribers
        # Subscribe to visual and thermal visualization images
        self.create_subscription(Image, self.visual_topic, self._visual_cb, 10)
        self.create_subscription(Image, self.thermal_topic, self._thermal_cb, 10)
        # Subscribe to raw temperature image (float32 per pixel, Celsius)
        self.create_subscription(Image, '/hospibot/thermal_raw', self._thermal_raw_cb, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_cb, 10)
        self.create_subscription(BoundingBox2D, self.bbox_topic, self._bbox_cb, 10)
        self.create_subscription(PointCloud2, self.radar_topic, self._radar_cb, 10)

        self.visual_img = None
        self.thermal_img = None
        self.thermal_raw = None  # float32 temperature map (H, W)
        self.camera_model = None
        self.last_bbox = None
        self.projected_points = []

        # Timer to refresh display
        self.create_timer(1.0 / 20.0, self._refresh)

        self.get_logger().info('ThermalViz node started.')

    def _visual_cb(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.visual_img = img
        except Exception as e:
            self.get_logger().error(f'visual img conversion failed: {e}')

    def _thermal_cb(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.thermal_img = img
        except Exception as e:
            self.get_logger().error(f'thermal img conversion failed: {e}')

    def _thermal_raw_cb(self, msg: Image):
        try:
            arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            # Ensure a 2D numpy float32 array
            if isinstance(arr, np.ndarray) and arr.dtype == np.float32:
                self.thermal_raw = arr
            else:
                self.get_logger().warn('Received thermal_raw with unexpected dtype/shape')
        except Exception as e:
            self.get_logger().error(f'thermal raw conversion failed: {e}')

    def _camera_info_cb(self, msg: CameraInfo):
        try:
            self.camera_model = PinholeCameraModel()
            self.camera_model.fromCameraInfo(msg)
            self.get_logger().info('Camera model initialized for visualization.')
        except Exception as e:
            self.get_logger().error(f'camera_info error: {e}')

    def _bbox_cb(self, msg: BoundingBox2D):
        # Convert center+size to pixel rectangle
        cx = int(msg.center.position.x)
        cy = int(msg.center.position.y)
        w = int(msg.size_x)
        h = int(msg.size_y)
        x1 = max(0, cx - w // 2)
        y1 = max(0, cy - h // 2)
        x2 = x1 + w
        y2 = y1 + h
        self.last_bbox = (x1, y1, x2, y2)

    def _radar_cb(self, msg: PointCloud2):
        # If we have a camera model, project points into image
        self.projected_points = []
        if self.camera_model is None:
            return
        try:
            gen = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            for p in gen:
                try:
                    u, v = self.camera_model.project3dToPixel(p)
                    if np.isfinite(u) and np.isfinite(v):
                        self.projected_points.append((int(u), int(v)))
                except Exception:
                    pass
        except Exception as e:
            self.get_logger().error(f'read_points failed: {e}')

    def _refresh(self):
        # Build side-by-side canvas
        if self.visual_img is None and self.thermal_img is None:
            return

        left = self.visual_img if self.visual_img is not None else np.zeros((480, 640, 3), dtype=np.uint8)
        right = self.thermal_img if self.thermal_img is not None else np.zeros_like(left)

        # Resize to same height
        h = min(left.shape[0], right.shape[0])
        # Maintain widths proportionally
        left_r = cv2.resize(left, (left.shape[1], h))
        right_r = cv2.resize(right, (right.shape[1], h))

        disp = np.hstack((left_r, right_r))

        # Overlay bbox on left and right (if present)
        left_w = left_r.shape[1]
        orig_h, orig_w = None, None
        if self.thermal_raw is not None:
            orig_h, orig_w = self.thermal_raw.shape[0], self.thermal_raw.shape[1]
        elif self.visual_img is not None:
            orig_h, orig_w = self.visual_img.shape[0], self.visual_img.shape[1]

        if self.last_bbox is not None and orig_w is not None:
            x1, y1, x2, y2 = self.last_bbox
            # Compute scale factors from original image -> display
            sx = left_r.shape[1] / float(orig_w)
            sy = left_r.shape[0] / float(orig_h)
            sx1, sy1 = int(x1 * sx), int(y1 * sy)
            sx2, sy2 = int(x2 * sx), int(y2 * sy)

            # Draw on left (visual)
            cv2.rectangle(disp, (sx1, sy1), (sx2, sy2), (0, 255, 0), 2)
            # Also draw on right (offset by left width)
            cv2.rectangle(disp, (left_w + sx1, sy1), (left_w + sx2, sy2), (0, 255, 0), 2)

            # Compute max temperature inside bbox (using raw temp map) and publish + display
            if self.thermal_raw is not None:
                # Clamp coordinates to bounds
                rx1 = max(0, min(orig_w - 1, x1))
                ry1 = max(0, min(orig_h - 1, y1))
                rx2 = max(0, min(orig_w, x2))
                ry2 = max(0, min(orig_h, y2))

                if rx2 > rx1 and ry2 > ry1:
                    region = self.thermal_raw[ry1:ry2, rx1:rx2]
                    if region.size > 0:
                        max_temp = float(np.nanmax(region))
                        # Publish numeric value
                        try:
                            from std_msgs.msg import Float32
                            temp_msg = Float32()
                            temp_msg.data = max_temp
                            self.temp_pub.publish(temp_msg)
                        except Exception:
                            pass

                        # Draw text on both halves
                        text = f"MaxT: {max_temp:.1f}C"
                        cv2.putText(disp, text, (sx1, max(15, sy1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        cv2.putText(disp, text, (left_w + sx1, max(15, sy1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Draw projected points (only if camera_model exists)
        if self.projected_points and orig_w is not None:
            sx = left_r.shape[1] / float(orig_w)
            sy = left_r.shape[0] / float(orig_h)
            for (u, v) in self.projected_points:
                pu = int(u * sx)
                pv = int(v * sy)
                cv2.circle(disp, (pu, pv), 3, (0, 0, 255), -1)
                cv2.circle(disp, (left_w + pu, pv), 3, (0, 0, 255), -1)

        # Publish composed visualization if requested
        if self.publish_viz and self.image_pub is not None:
            try:
                viz_msg = self.bridge.cv2_to_imgmsg(disp, encoding='bgr8')
                viz_msg.header.stamp = self.get_clock().now().to_msg()
                viz_msg.header.frame_id = 'thermal_camera_link'
                self.image_pub.publish(viz_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish viz image: {e}')

        # Optionally show a debug window (default off)
        if self.show_window:
            cv2.imshow(self.window_name, disp)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ThermalViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
