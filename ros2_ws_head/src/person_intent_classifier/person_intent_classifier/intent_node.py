#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from vision_msgs.msg import BoundingBox2D, Point2D
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Float32 
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry

from collections import deque
import numpy as np
import yaml
import math
from pathlib import Path


class StateStabilizer:
    def __init__(self, persistence_thresh=10):
        self.history = deque(maxlen=persistence_thresh)
        self.current_state = "UNKNOWN"

    def update(self, new_state):
        self.history.append(new_state)
        
        # Only update if the buffer is full enough
        if len(self.history) == self.history.maxlen:
            # Find the most common state in the recent history
            most_common = max(set(self.history), key=self.history.count)
            self.current_state = most_common
            
        return self.current_state


class PersonIntentNode(Node):
    def __init__(self):
        super().__init__('person_intent_classifier')

        self._load_configuration()

        # ----------------------------
        # Parameters
        # ----------------------------
        self.declare_parameter('bbox_topic', self.config['bbox_topic'])
        self.declare_parameter('radar_topic', self.config['radar_topic'])
        self.declare_parameter('odom_topic', self.config.get('odom_topic', '/odom'))

        self.declare_parameter('image_width', self.config['image_width'])
        self.declare_parameter('image_height', self.config['image_height'])

        self.declare_parameter('interact_threshold_ratio', self.config['interact_threshold_ratio'])
        self.declare_parameter('close_threshold_ratio', self.config['close_threshold_ratio'])
        self.declare_parameter('interaction_radius', self.config['interaction_radius'])
        self.declare_parameter('radar_timeout', self.config['radar_timeout'])

        self.declare_parameter('persistence_threshold', self.config['persistence_threshold'])
        self.declare_parameter('smoothing_window', self.config['smoothing_window'])
        self.declare_parameter('radar_smoothing_window', self.config['radar_smoothing_window'])

        self.declare_parameter('low_speed_threshold', self.config['low_speed_threshold'])
        self.declare_parameter('moving_away_threshold', self.config['moving_away_threshold'])
        self.declare_parameter('min_time_delta', self.config['min_time_delta'])

        self.declare_parameter('approaching_threshold', self.config['approaching_threshold'])
        self.declare_parameter('moving_away_size_threshold', self.config['moving_away_size_threshold'])

        self.declare_parameter('bbox_timeout', self.config.get('bbox_timeout', 2.0))

        # Radar range axis & offset
        self.declare_parameter('radar_range_axis', self.config.get('radar_range_axis', 'z'))
        self.declare_parameter('radar_max_range', self.config.get('radar_max_range', 10.0))
        self.declare_parameter('radar_forward_offset', self.config.get('radar_forward_offset', 0.05))

        self._setup_parameters()

        # ----------------------------
        # State
        # ----------------------------
        self.history_size_x = deque(maxlen=self.SMOOTHING_WINDOW)
        self.history_size_y = deque(maxlen=self.SMOOTHING_WINDOW)

        self.prev_cam_avg_w = None
        self.prev_cam_avg_h = None

        self.latest_eye_center = None

        # Odometry State
        self.robot_v_x = 0.0
        self.robot_omega_z = 0.0

        # Radar state
        self.radar_active = False
        self.last_radar_time = 0.0

        self.radar_centroid_history = deque(maxlen=self.radar_smoothing_window)
        self.dist_history = deque(maxlen=self.radar_smoothing_window)

        self.prev_radar_centroid = None
        self.prev_radar_timestamp = None
        self.prev_radar_dist = None

        self.latest_intent = "UNKNOWN"
        self.latest_metrics = ""

        self.last_bbox_time = 0.0
        self.bbox_timeout = float(self.get_parameter('bbox_timeout').value)

        # Stabilizer
        self.stabilizer = StateStabilizer(
            persistence_thresh=int(self.get_parameter('persistence_threshold').value)
        )

        # Current intent tracking
        self.current_stable_intent = "UNKNOWN"

        # ----------------------------
        # ROS interfaces
        # ----------------------------
        self.create_subscription(
            BoundingBox2D,
            self.get_parameter('bbox_topic').value,
            self.camera_callback,
            10
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter('radar_topic').value,
            self.radar_callback,
            10
        )
        self.create_subscription(
            Point2D,
            '/face_tracker/eye_center',
            self.eye_center_callback,
            10
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            10
        )

        self.intent_publisher = self.create_publisher(String, '/person_intent', 10)
        
        # Publisher for the radar velocity
        self.velocity_publisher = self.create_publisher(Float32, '/person_intent/radar_velocity', 10)

        self.create_timer(0.2, self.status_check_callback)

        self.get_logger().info("Intent Node Started with Tiered Size Logic & Stabilized Radar")

    # ----------------------------
    # Config
    # ----------------------------
    def _load_configuration(self):
        default_config = {
            'bbox_topic': '/person_bounding_box',
            'radar_topic': '/person_detect/filtered_points',
            'odom_topic': '/odom',
            'image_width': 256,
            'image_height': 192,
            'interact_threshold_ratio': 0.60, # NEW: Area > 60% means WANT_TO_INTERACT
            'close_threshold_ratio': 0.40,    # Area > 40% means CLOSE_PROXIMITY
            'interaction_radius': 2.0,
            'radar_timeout': 0.5,
            'persistence_threshold': 12,
            'smoothing_window': 5,
            'radar_smoothing_window': 5,
            'low_speed_threshold': 0.08,
            'moving_away_threshold': 0.12,
            'min_time_delta': 0.10,
            'approaching_threshold': 0.8,
            'moving_away_size_threshold': 0.8,
            'bbox_timeout': 2.0,
            'radar_range_axis': 'z',
            'radar_max_range': 10.0,
            'radar_forward_offset': 0.05,
        }

        try:
            import ament_index_python
            share_dir = ament_index_python.get_package_share_directory('person_intent_classifier')
            yaml_path = Path(share_dir) / 'config' / 'intent_classifier_params.yaml'
        except Exception:
            yaml_path = Path(__file__).parent.parent / 'config' / 'intent_classifier_params.yaml'

        if yaml_path.exists():
            try:
                with open(yaml_path, 'r') as file:
                    yaml_data = yaml.safe_load(file)
                    if '/person_intent_classifier' in yaml_data and 'ros__parameters' in yaml_data['/person_intent_classifier']:
                        params = yaml_data['/person_intent_classifier']['ros__parameters']
                        self.config = {**default_config, **params}
                    else:
                        self.config = default_config
            except Exception:
                self.config = default_config
        else:
            self.config = default_config

    def _setup_parameters(self):
        self.IMAGE_WIDTH = float(self.get_parameter('image_width').value)
        self.IMAGE_HEIGHT = float(self.get_parameter('image_height').value)
        self.IMAGE_AREA = self.IMAGE_WIDTH * self.IMAGE_HEIGHT

        self.INTERACT_THRESH_RATIO = float(self.get_parameter('interact_threshold_ratio').value)
        self.CLOSE_THRESH_RATIO = float(self.get_parameter('close_threshold_ratio').value)
        self.INTERACTION_RADIUS = float(self.get_parameter('interaction_radius').value)
        self.RADAR_TIMEOUT = float(self.get_parameter('radar_timeout').value)

        self.SMOOTHING_WINDOW = int(self.get_parameter('smoothing_window').value)
        self.radar_smoothing_window = int(self.get_parameter('radar_smoothing_window').value)

        self.LOW_SPEED_THRESHOLD = float(self.get_parameter('low_speed_threshold').value)
        self.MOVING_AWAY_THRESHOLD = float(self.get_parameter('moving_away_threshold').value)
        self.MIN_TIME_DELTA = float(self.get_parameter('min_time_delta').value)

        self.APPROACHING_THRESHOLD = float(self.get_parameter('approaching_threshold').value)
        self.MOVING_AWAY_SIZE_THRESHOLD = float(self.get_parameter('moving_away_size_threshold').value)

        self.RADAR_RANGE_AXIS = str(self.get_parameter('radar_range_axis').value).lower()
        self.RADAR_MAX_RANGE = float(self.get_parameter('radar_max_range').value)
        self.RADAR_FORWARD_OFFSET = float(self.get_parameter('radar_forward_offset').value)

    # ----------------------------
    # Helpers
    # ----------------------------
    def _pc2_to_xyz(self, msg: PointCloud2) -> np.ndarray:
        try:
            arr = pc2.read_points_numpy(msg, field_names=("x", "y", "z"))
            xyz = np.column_stack((arr['x'], arr['y'], arr['z'])).astype(np.float32, copy=False)
            return xyz
        except Exception:
            pts = []
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                if isinstance(p, (tuple, list)):
                    pts.append([p[0], p[1], p[2]])
                else:
                    pts.append([p['x'], p['y'], p['z']])
            return np.asarray(pts, dtype=np.float32)

    def _range_from_centroid(self, centroid: np.ndarray) -> float:
        if self.RADAR_RANGE_AXIS == 'x':
            return float(abs(centroid[0]))
        if self.RADAR_RANGE_AXIS == 'y':
            return float(abs(centroid[1]))
        if self.RADAR_RANGE_AXIS == 'norm':
            return float(np.linalg.norm(centroid))
        return float(abs(centroid[2]))

    # ----------------------------
    # Odometry
    # ----------------------------
    def odom_callback(self, msg: Odometry):
        self.robot_v_x = msg.twist.twist.linear.x
        self.robot_omega_z = msg.twist.twist.angular.z

    # ----------------------------
    # Eye center
    # ----------------------------
    def eye_center_callback(self, msg):
        self.latest_eye_center = msg

    # ----------------------------
    # Radar 
    # ----------------------------
    def radar_callback(self, msg: PointCloud2):
        xyz = self._pc2_to_xyz(msg)
        if xyz.size == 0:
            return

        # Remove nonsense ranges
        ranges = np.linalg.norm(xyz, axis=1)
        valid_mask = (ranges > 0.05) & (ranges < self.RADAR_MAX_RANGE)
        xyz = xyz[valid_mask]
        
        if xyz.size == 0:
            return

        # Average the entire filtered point cloud directly
        centroid_raw = np.mean(xyz, axis=0)

        # Smooth centroid
        self.radar_centroid_history.append(centroid_raw)
        centroid = np.mean(self.radar_centroid_history, axis=0)

        # Distance from centroid
        dist_raw = self._range_from_centroid(centroid)
        self.dist_history.append(dist_raw)
        dist = float(np.mean(self.dist_history))

        # Use message stamp for dt stability
        t = Time.from_msg(msg.header.stamp).nanoseconds / 1e9

        if self.prev_radar_timestamp is None:
            self.prev_radar_timestamp = t
            self.prev_radar_centroid = centroid
            self.prev_radar_dist = dist
            self.last_radar_time = self.get_clock().now().nanoseconds / 1e9
            self.radar_active = True
            return

        dt = t - self.prev_radar_timestamp
        
        # --- FIX: Detect ROS bag loop (time jumping backwards) ---
        if dt < 0.0:
            self.get_logger().warn("Time jumped backwards (Bag looped). Resetting radar state.")
            self.prev_radar_timestamp = t
            self.prev_radar_centroid = centroid
            self.prev_radar_dist = dist
            
            # Clear smoothing histories so the person doesn't "teleport"
            self.radar_centroid_history.clear()
            self.dist_history.clear()
            return
        # ---------------------------------------------------------

        if dt < self.MIN_TIME_DELTA:
            return

        # Raw Range-rate: + away, - approach (RELATIVE VELOCITY)
        r_dot_raw = (dist - self.prev_radar_dist) / dt

        # Outlier rejection for jitter spikes
        if abs(r_dot_raw) > 6.0:
            self.prev_radar_timestamp = t
            return

        # --- EGO-MOTION COMPENSATION ---
        theta = math.atan2(centroid[0], centroid[2])
        
        v_comp_forward = self.robot_v_x * math.cos(theta)
        v_radar_lateral = self.robot_omega_z * self.RADAR_FORWARD_OFFSET
        v_comp_lateral = v_radar_lateral * math.sin(theta)
        
        robot_radial_vel = v_comp_forward - v_comp_lateral
        r_dot = r_dot_raw + robot_radial_vel
        # -------------------------------

        # Publish the velocity for analytics
        vel_msg = Float32()
        vel_msg.data = float(r_dot)
        self.velocity_publisher.publish(vel_msg)

        # Decide intent based on compensated r_dot
        metrics = f"Dist: {dist:.2f}m | r_dot: {r_dot:.2f}m/s"
        if abs(r_dot) < self.LOW_SPEED_THRESHOLD:
            if dist < 0.5:
                intent = "WANT_TO_INTERACT"
            else:
                intent = "STATIONARY"
        elif r_dot > self.MOVING_AWAY_THRESHOLD:
            intent = "MOVING_AWAY"
            metrics += " (Away)"
        else:
            if dist < self.INTERACTION_RADIUS:
                intent = "CLOSE_PROXIMITY"
            else:
                intent = "PASSING_BY"
            metrics += " (Approach)"

        self.latest_intent = intent
        self.latest_metrics = f"[RADAR] {metrics}"

        self.prev_radar_timestamp = t
        self.prev_radar_centroid = centroid
        self.prev_radar_dist = dist
        
        self.last_radar_time = self.get_clock().now().nanoseconds / 1e9
        self.radar_active = True

    # ----------------------------
    # Camera
    # ----------------------------
    def camera_callback(self, msg: BoundingBox2D):
        self.last_bbox_time = self.get_clock().now().nanoseconds / 1e9

        now = self.get_clock().now().nanoseconds / 1e9
        is_radar_fresh = (now - self.last_radar_time) < self.RADAR_TIMEOUT

        bbox_w = msg.size_x
        bbox_h = msg.size_y

        # Out-of-bounds override
        if bbox_w > self.IMAGE_WIDTH or bbox_h > self.IMAGE_HEIGHT:
            bbox_w = min(bbox_w, self.IMAGE_WIDTH)
            bbox_h = min(bbox_h, self.IMAGE_HEIGHT)

        self.history_size_x.append(bbox_w)
        self.history_size_y.append(bbox_h)

        if len(self.history_size_y) < self.SMOOTHING_WINDOW:
            return

        curr_w = float(np.mean(self.history_size_x))
        curr_h = float(np.mean(self.history_size_y))

        if self.prev_cam_avg_h is None:
            self.prev_cam_avg_w = curr_w
            self.prev_cam_avg_h = curr_h
            return

        final_intent = "UNKNOWN"
        debug_str = ""

        curr_area = curr_w * curr_h
        area_ratio = curr_area / max(1.0, self.IMAGE_AREA)
        tall_ratio = curr_h / self.IMAGE_HEIGHT

        # Tier 1: Visual Proximity
        if area_ratio > self.INTERACT_THRESH_RATIO:
            final_intent = "WANT_TO_INTERACT"
            debug_str = f"Visual Tier 1: INTERACT (Area: {area_ratio*100:.0f}%, Tall: {tall_ratio*100:.0f}%)"

        # Tier 2: Visual Proximity
        elif area_ratio > self.CLOSE_THRESH_RATIO:
            final_intent = "CLOSE_PROXIMITY"
            debug_str = f"Visual Tier 2: CLOSE (Area: {area_ratio*100:.0f}%, Tall: {tall_ratio*100:.0f}%)"

        # Priority 2: Radar overrides small camera movements
        elif is_radar_fresh and self.radar_active:
            final_intent = self.latest_intent
            debug_str = self.latest_metrics

        # Priority 3: Camera movement fallback
        else:
            delta_h = curr_h - self.prev_cam_avg_h
            
            if delta_h > self.APPROACHING_THRESHOLD:
                final_intent = "CLOSE_PROXIMITY"
                debug_str = "[CAM-ONLY] Approaching"
                
            elif delta_h < -self.MOVING_AWAY_SIZE_THRESHOLD:
                final_intent = "MOVING_AWAY"
                debug_str = "[CAM-ONLY] Moving Away"
                
            else:
                final_intent = "STATIONARY"
                debug_str = "[CAM-ONLY] Static/Far"

        stable_intent = self.stabilizer.update(final_intent)
        self.current_stable_intent = stable_intent

        self.prev_cam_avg_w = curr_w
        self.prev_cam_avg_h = curr_h

        self.get_logger().info(f"Intent: {stable_intent} | {debug_str}")

        out = String()
        out.data = stable_intent
        self.intent_publisher.publish(out)

    # ----------------------------
    # Status check
    # ----------------------------
    def status_check_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self.last_bbox_time) > self.bbox_timeout:
            self.current_stable_intent = "NO_PERSON_DETECTED"
            self.get_logger().info("Intent: NO_PERSON_DETECTED | No bounding box received")
            out = String()
            out.data = "NO_PERSON_DETECTED"
            self.intent_publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PersonIntentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
