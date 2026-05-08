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
        # New: publisher for processed thermal image (visualized)
        self.thermal_pub = self.create_publisher(Image, '/hospibot/thermal_image', 10)
        # New: publisher for raw temperature values (float32, Celsius) aligned with the thermal image
        self.thermal_raw_pub = self.create_publisher(Image, '/hospibot/thermal_raw', 10)
        self.pub = self.create_publisher(CameraInfo, '/hospibot/camera_info', 10)
        self.bridge = CvBridge()
        # Try to open the thermal camera and request raw (packed) frames when possible
        self.cap = cv2.VideoCapture('/dev/video2')  # Open the thermalcamera
        try:
            # Prefer raw frames (no automatic RGB conversion) so we can extract thermal data
            self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
            # Request expected resolution that contains both visual + thermal halves
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 256)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 384)
            self.cap.set(cv2.CAP_PROP_FPS, 25)
        except Exception:
            pass
        
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

        stamp = self.get_clock().now().to_msg()

        # Case A: driver already returns a 3-channel BGR frame (backwards compatible)
        if hasattr(frame, 'ndim') and frame.ndim == 3 and frame.shape[2] == 3:
            height = frame.shape[0] // 2
            top_half = frame[:height, :]
            top_flipped = cv2.flip(top_half, 0)
            top_flipped = cv2.flip(top_flipped, 1)

            try:
                ros_image = self.bridge.cv2_to_imgmsg(top_flipped, encoding="bgr8")
                ros_image.header.stamp = stamp
                ros_image.header.frame_id = "thermal_camera_link"
                self.det_pub.publish(ros_image)
            except Exception as e:
                self.get_logger().error(f"Failed to publish visual image: {e}")

            # Try to publish a simple thermal visualization from bottom half if available (best-effort)
            try:
                bottom_half = frame[height:, :]
                # Convert bottom half to grayscale and apply a colormap for visualization
                gray = cv2.cvtColor(bottom_half, cv2.COLOR_BGR2GRAY)
                thermal_vis = cv2.applyColorMap(gray, cv2.COLORMAP_JET)
                thermal_vis = cv2.flip(thermal_vis, 0)
                thermal_vis = cv2.flip(thermal_vis, 1)
                th_msg = self.bridge.cv2_to_imgmsg(thermal_vis, encoding="bgr8")
                th_msg.header.stamp = stamp
                th_msg.header.frame_id = "thermal_camera_link"
                self.thermal_pub.publish(th_msg)
            except Exception:
                # Not critical — best-effort only
                pass

            # Publish CameraInfo unchanged
            try:
                self.msg.header.stamp = stamp
                self.msg.header.frame_id = "thermal_camera_link"
                self.pub.publish(self.msg)
            except Exception as e:
                self.get_logger().error(f"Failed to publish camera info: {e}")

            return

        # Case B: raw packed frames (e.g. YUYV-like / P2 Pro raw) — follow the sample processing
        try:
            flat_data = frame.flatten()
        except Exception as e:
            self.get_logger().error(f"Unexpected frame format: {e}")
            return

        imdata = None
        thdata = None

        # Detect packed sizes: 256x384x2 or 256x192x2
        if flat_data.size == 256 * 384 * 2:
            reshaped = flat_data.reshape(384, 256, 2)
            imdata, thdata = np.array_split(reshaped, 2)
        elif flat_data.size == 256 * 192 * 2:
            imdata = flat_data.reshape(192, 256, 2)
        else:
            # Unknown format — try falling back to previous splitting method
            if hasattr(frame, 'shape') and frame.shape[0] >= 2:
                height = frame.shape[0] // 2
                top_half = frame[:height, :]
                top_flipped = cv2.flip(top_half, 0)
                top_flipped = cv2.flip(top_flipped, 1)
                try:
                    ros_image = self.bridge.cv2_to_imgmsg(top_flipped, encoding="bgr8")
                    ros_image.header.stamp = stamp
                    ros_image.header.frame_id = "thermal_camera_link"
                    self.det_pub.publish(ros_image)
                except Exception:
                    pass
                try:
                    self.msg.header.stamp = stamp
                    self.msg.header.frame_id = "thermal_camera_link"
                    self.pub.publish(self.msg)
                except Exception:
                    pass
            return

        # --- Process visual image (imdata) ---
        try:
            # Convert packed YUYV-like data to BGR
            bgr = cv2.cvtColor(imdata, cv2.COLOR_YUV2BGR_YUYV)
            # Ensure same flipping as existing behavior
            bgr = cv2.flip(bgr, 0)
            bgr = cv2.flip(bgr, 1)

            ros_image = self.bridge.cv2_to_imgmsg(bgr, encoding="bgr8")
            ros_image.header.stamp = stamp
            ros_image.header.frame_id = "thermal_camera_link"
            self.det_pub.publish(ros_image)

            # Publish camera info with same stamp
            self.msg.header.stamp = stamp
            self.msg.header.frame_id = "thermal_camera_link"
            self.pub.publish(self.msg)
        except Exception as e:
            self.get_logger().error(f"Failed to process visual half: {e}")

        # --- Process thermal half (thdata) if present ---
        if thdata is not None:
            try:
                # thdata is shape (192,256,2): channel0=low byte, channel1=high byte (per sample)
                # Build a temperature map in Celsius
                lo = thdata[:, :, 0].astype(np.uint16)
                hi = thdata[:, :, 1].astype(np.uint16)
                raw = lo + (hi << 8)
                temp_c = (raw.astype(np.float32) / 64.0) - 273.15

                # Normalize to 0-255 for visualization (clip to sensible range)
                min_t = np.nanmin(temp_c)
                max_t = np.nanmax(temp_c)
                if max_t - min_t < 1e-3:
                    min_t = temp_c.min() - 1.0
                    max_t = temp_c.max() + 1.0
                norm = ((temp_c - min_t) / (max_t - min_t) * 255.0).astype(np.uint8)

                # Apply colormap
                thermal_vis = cv2.applyColorMap(norm, cv2.COLORMAP_JET)

                # Resize/flip to match visual processing (and orientation)
                thermal_vis = cv2.flip(thermal_vis, 0)
                thermal_vis = cv2.flip(thermal_vis, 1)

                # Publish thermal visualization
                th_msg = self.bridge.cv2_to_imgmsg(thermal_vis, encoding="bgr8")
                th_msg.header.stamp = stamp
                th_msg.header.frame_id = "thermal_camera_link"
                self.thermal_pub.publish(th_msg)

                # Publish raw temperature map (float32, Celsius) aligned with visualization
                try:
                    temp32 = temp_c.astype(np.float32)
                    # Apply same flips as visualization so coordinates align with visual image
                    temp32 = cv2.flip(temp32, 0)
                    temp32 = cv2.flip(temp32, 1)
                    raw_msg = self.bridge.cv2_to_imgmsg(temp32, encoding="32FC1")
                    raw_msg.header.stamp = stamp
                    raw_msg.header.frame_id = "thermal_camera_link"
                    self.thermal_raw_pub.publish(raw_msg)
                except Exception as e:
                    self.get_logger().error(f"Failed to publish thermal raw: {e}")

            except Exception as e:
                self.get_logger().error(f"Failed to process thermal half: {e}")

    
        
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
