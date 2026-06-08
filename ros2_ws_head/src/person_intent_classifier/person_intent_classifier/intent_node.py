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
        
        if len(self.history) == self.history.maxlen:
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
        self.declare_parameter('use_odom', self.config.get('use_odom', False))

        self.declare_parameter('image_width', self.config['image_width'])
        self.declare_parameter('image_height', self.config['image_height'])

        self.declare_parameter('interact_threshold_ratio', self.config['interact_threshold_ratio'])
        self.declare_parameter('close_threshold_ratio', self.config['close_threshold_ratio'])
        
        self.declare_parameter('interact_distance_thresh', self.config['interact_distance_thresh'])
        self.declare_parameter('close_distance_thresh', self.config['close_distance_thresh'])
        
        self.declare_parameter('radar_timeout', self.config['radar_timeout'])

        self.declare_parameter('persistence_threshold', self.config['persistence_threshold'])
        self.declare_parameter('smoothing_window', self.config['smoothing_window'])
        self.declare_parameter('radar_smoothing_window', self.config['radar_smoothing_window'])

        self.declare_parameter('low_speed_threshold', self.config['low_speed_threshold'])
        self.declare_parameter('moving_away_size_threshold', self.config.get('moving_away_size_threshold', 0.8))
        self.declare_parameter('min_time_delta', self.config['min_time_delta'])

        self.declare_parameter('bbox_timeout', self.config.get('bbox_timeout', 2.0))

        self.declare_parameter('radar_range_axis', self.config.get('radar_range_axis', 'z'))
        self.declare_parameter('radar_max_range', self.config.get('radar_max_range', 10.0))
        self.declare_parameter('radar_forward_offset', self.config.get('radar_forward_offset', 0.05))
        self.declare_parameter('hysteresis_distance_buffer', self.config.get('hysteresis_distance_buffer', 0.2))

        self._setup_parameters()

        # ----------------------------
        # State
        # ----------------------------
        self.history_size_x = deque(maxlen=self.SMOOTHING_WINDOW)
        self.history_size_y = deque(maxlen=self.SMOOTHING_WINDOW)

        self.prev_cam_avg_w = None
        self.prev_cam_avg_h = None

        self.latest_eye_center = None

        self.robot_v_x = 0.0
        self.robot_omega_z = 0.0

        self.radar_active = False
        self.last_radar_time = 0.0

        self.radar_centroid_history = deque(maxlen=self.radar_smoothing_window)
        self.dist_history = deque(maxlen=self.radar_smoothing_window)

        self.prev_radar_centroid = None
        self.prev_radar_timestamp = None
        self.prev_radar_dist = None
        
        self.latest_r_dot = 0.0
        self.latest_dist = 0.0
        self.latest_dca = 0.0  

        self.last_bbox_time = 0.0
        self.bbox_timeout = float(self.get_parameter('bbox_timeout').value)

        self.stabilizer = StateStabilizer(
            persistence_thresh=int(self.get_parameter('persistence_threshold').value)
        )

        self.current_stable_intent = "UNKNOWN"

        # ----------------------------
        # ROS interfaces
        # ----------------------------
        self.create_subscription(BoundingBox2D, self.get_parameter('bbox_topic').value, self.camera_callback, 10)
        self.create_subscription(PointCloud2, self.get_parameter('radar_topic').value, self.radar_callback, 10)
        self.create_subscription(Point2D, '/face_tracker/eye_center', self.eye_center_callback, 10)
        if self.USE_ODOM:
            self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.odom_callback, 10)
            self.get_logger().info("Odometry usage is ENABLED.")
        else:
            self.get_logger().info("Odometry usage is DISABLED. Assuming robot is stationary.")

        self.intent_publisher = self.create_publisher(String, '/person_intent', 10)
        self.velocity_publisher = self.create_publisher(Float32, '/person_intent/radar_velocity', 10)
        self.distance_publisher = self.create_publisher(Float32, '/person_intent/radar_distance', 10)

        self.create_timer(0.2, self.status_check_callback)
        self.get_logger().info("Intent Node Started with Hysteresis Anti-Flicker Logic")

    # ----------------------------
    # Config
    # ----------------------------
    def _load_configuration(self):
        default_config = {
            'bbox_topic': '/person_bounding_box',
            'radar_topic': '/person_detect/filtered_points',
            'odom_topic': '/odom',
            'use_odom': False,
            'image_width': 256,
            'image_height': 192,
            'interact_threshold_ratio': 0.30, 
            'close_threshold_ratio': 0.20,    
            'interact_distance_thresh': 1.8, 
            'close_distance_thresh': 3.2,    
            'radar_timeout': 0.5,
            'persistence_threshold': 10,
            'smoothing_window': 5,
            'radar_smoothing_window': 5,
            'low_speed_threshold': 0.08,
            'moving_away_size_threshold': 0.8,
            'min_time_delta': 0.10,
            'bbox_timeout': 2.0,
            'radar_range_axis': 'z',
            'radar_max_range': 10.0,
            'radar_forward_offset': 0.05,
            'hysteresis_distance_buffer': 0.2,
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
        self.INTERACT_DIST = float(self.get_parameter('interact_distance_thresh').value)
        self.CLOSE_DIST = float(self.get_parameter('close_distance_thresh').value)
        self.RADAR_TIMEOUT = float(self.get_parameter('radar_timeout').value)

        self.SMOOTHING_WINDOW = int(self.get_parameter('smoothing_window').value)
        self.radar_smoothing_window = int(self.get_parameter('radar_smoothing_window').value)

        self.LOW_SPEED_THRESHOLD = float(self.get_parameter('low_speed_threshold').value)
        self.MOVING_AWAY_SIZE_THRESHOLD = float(self.get_parameter('moving_away_size_threshold').value)
        self.MIN_TIME_DELTA = float(self.get_parameter('min_time_delta').value)

        self.RADAR_RANGE_AXIS = str(self.get_parameter('radar_range_axis').value).lower()
        self.RADAR_MAX_RANGE = float(self.get_parameter('radar_max_range').value)
        self.RADAR_FORWARD_OFFSET = float(self.get_parameter('radar_forward_offset').value)
        self.HYSTERESIS_BUFFER = float(self.get_parameter('hysteresis_distance_buffer').value)
        self.USE_ODOM = bool(self.get_parameter('use_odom').value)

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
        if self.RADAR_RANGE_AXIS == 'x': return float(abs(centroid[0]))
        if self.RADAR_RANGE_AXIS == 'y': return float(abs(centroid[1]))
        if self.RADAR_RANGE_AXIS == 'norm': return float(np.linalg.norm(centroid))
        return float(abs(centroid[2]))

    def odom_callback(self, msg: Odometry):
        self.robot_v_x = msg.twist.twist.linear.x
        self.robot_omega_z = msg.twist.twist.angular.z

    def eye_center_callback(self, msg):
        self.latest_eye_center = msg

    # ----------------------------
    # Radar
    # ----------------------------
    def radar_callback(self, msg: PointCloud2):
        xyz = self._pc2_to_xyz(msg)
        if xyz.size == 0: return

        ranges = np.linalg.norm(xyz, axis=1)
        valid_mask = (ranges > 0.05) & (ranges < self.RADAR_MAX_RANGE)
        xyz = xyz[valid_mask]
        
        if xyz.size == 0: return

        centroid_raw = np.mean(xyz, axis=0)
        self.radar_centroid_history.append(centroid_raw)
        centroid = np.mean(self.radar_centroid_history, axis=0)

        dist_raw = self._range_from_centroid(centroid)
        self.dist_history.append(dist_raw)
        dist = float(np.mean(self.dist_history))

        t = Time.from_msg(msg.header.stamp).nanoseconds / 1e9

        if self.prev_radar_timestamp is None:
            self.prev_radar_timestamp = t
            self.prev_radar_centroid = centroid
            self.prev_radar_dist = dist
            self.last_radar_time = self.get_clock().now().nanoseconds / 1e9
            self.radar_active = True
            return

        dt = t - self.prev_radar_timestamp
        
        if dt < 0.0:
            self.prev_radar_timestamp = t
            self.prev_radar_centroid = centroid
            self.prev_radar_dist = dist
            self.radar_centroid_history.clear()
            self.dist_history.clear()
            return

        if dt < self.MIN_TIME_DELTA: return

        r_dot_raw = (dist - self.prev_radar_dist) / dt
        if abs(r_dot_raw) > 6.0:
            self.prev_radar_timestamp = t
            return

        dx = centroid[0] - self.prev_radar_centroid[0]
        dz = centroid[2] - self.prev_radar_centroid[2]
        delta_mag = math.hypot(dx, dz)

        if delta_mag < 0.01: 
            dca = dist
        else:
            dot_prod = centroid[0] * dx + centroid[2] * dz
            if dot_prod > 0:
                dca = dist
            else:
                dca = abs(centroid[0] * dz - centroid[2] * dx) / delta_mag

        theta = math.atan2(centroid[0], centroid[2])
        v_comp_forward = self.robot_v_x * math.cos(theta)
        v_radar_lateral = self.robot_omega_z * self.RADAR_FORWARD_OFFSET
        v_comp_lateral = v_radar_lateral * math.sin(theta)
        
        robot_radial_vel = v_comp_forward - v_comp_lateral
        r_dot = r_dot_raw + robot_radial_vel

        vel_msg = Float32()
        vel_msg.data = float(r_dot)
        self.velocity_publisher.publish(vel_msg)

        dist_msg = Float32()
        dist_msg.data = float(dist)
        self.distance_publisher.publish(dist_msg)

        self.latest_dist = dist
        self.latest_r_dot = r_dot
        self.latest_dca = dca

        self.prev_radar_timestamp = t
        self.prev_radar_centroid = centroid
        self.prev_radar_dist = dist
        self.last_radar_time = self.get_clock().now().nanoseconds / 1e9
        self.radar_active = True

    # ----------------------------
    # Camera / Fusion Entry Point
    # ----------------------------
    def camera_callback(self, msg: BoundingBox2D):
        self.last_bbox_time = self.get_clock().now().nanoseconds / 1e9
        now = self.get_clock().now().nanoseconds / 1e9
        is_radar_fresh = (now - self.last_radar_time) < self.RADAR_TIMEOUT

        bbox_w = msg.size_x
        bbox_h = msg.size_y

        if bbox_w > self.IMAGE_WIDTH or bbox_h > self.IMAGE_HEIGHT:
            bbox_w = min(bbox_w, self.IMAGE_WIDTH)
            bbox_h = min(bbox_h, self.IMAGE_HEIGHT)

        self.history_size_x.append(bbox_w)
        self.history_size_y.append(bbox_h)

        if len(self.history_size_y) < self.SMOOTHING_WINDOW: return

        curr_w = float(np.mean(self.history_size_x))
        curr_h = float(np.mean(self.history_size_y))

        if self.prev_cam_avg_h is None:
            self.prev_cam_avg_w = curr_w
            self.prev_cam_avg_h = curr_h
            return

        final_intent = "UNKNOWN"
        debug_str = ""
        delta_h = curr_h - self.prev_cam_avg_h

        # ---------------------------------------------------------
        # PRIMARY LOGIC: SENSOR FUSION (Radar Physics + Visual Height)
        # ---------------------------------------------------------
        if is_radar_fresh and self.radar_active:
            r_dist = self.latest_dist
            r_vel = self.latest_r_dot
            dca = self.latest_dca
            
            # --- MOTION HYSTERESIS: Anti-Flicker for Moving Away ---
            is_leaving = False
            if self.current_stable_intent == "MOVING_AWAY":
                is_leaving = delta_h < -0.2 
            else:
                is_leaving = delta_h < -self.MOVING_AWAY_SIZE_THRESHOLD

            # --- SPATIAL HYSTERESIS: Anti-Boundary Flicker for Distances ---
            # We add a 0.2m "sticky" buffer to thresholds so standing exactly on the line doesn't flicker
            buffer = self.HYSTERESIS_BUFFER 
            
            active_interact_thresh = self.INTERACT_DIST
            if self.current_stable_intent == "WANT_TO_INTERACT":
                active_interact_thresh += buffer  # Pushes threshold out to 2.0m to prevent dropping state
                
            active_close_thresh = self.CLOSE_DIST
            if self.current_stable_intent in ["WANT_TO_INTERACT", "CLOSE_PROXIMITY"]:
                active_close_thresh += buffer     # Pushes threshold out to 3.4m to prevent dropping state
            # -------------------------------------------------------------

            # 1. Immediate Interaction
            if r_dist < active_interact_thresh:
                final_intent = "WANT_TO_INTERACT"
                debug_str = f"[FUSED] Dist: {r_dist:.2f}m < {active_interact_thresh:.1f}m"

            # 2. Universal Leaving (Overrides Close/Outer zones)
            elif is_leaving and r_dist > active_interact_thresh:
                final_intent = "MOVING_AWAY"
                debug_str = f"[FUSED] Box Shrinking. Dist: {r_dist:.2f}m, delta_h: {delta_h:.1f}px"

            # 3. Close Proximity
            elif r_dist < active_close_thresh:
                final_intent = "CLOSE_PROXIMITY"
                debug_str = f"[FUSED] Dist: {r_dist:.2f}m (Between {active_interact_thresh:.1f}m and {active_close_thresh:.1f}m)"

            # 4. Outer Zone (> 3.2m)
            else:
                if abs(r_vel) < self.LOW_SPEED_THRESHOLD:
                    final_intent = "STATIONARY"
                    debug_str = f"[FUSED] Stationary afar. Dist: {r_dist:.2f}m"
                else:
                    if dca > 1.8:
                        final_intent = "PASSING_BY"
                        debug_str = f"[FUSED] Passing By. Dist: {r_dist:.2f}m, DCA: {dca:.2f}m > 1.8m"
                    else:
                        final_intent = "APPROACHING"
                        debug_str = f"[FUSED] Approaching. Dist: {r_dist:.2f}m, DCA: {dca:.2f}m <= 1.8m"

        # ---------------------------------------------------------
        # FALLBACK LOGIC: CAMERA ONLY
        # ---------------------------------------------------------
        else:
            curr_area = curr_w * curr_h
            area_ratio = curr_area / max(1.0, self.IMAGE_AREA)

            # Mirror the Hysteresis Logic for the fallback
            is_leaving_fallback = False
            if self.current_stable_intent == "MOVING_AWAY":
                is_leaving_fallback = delta_h < -0.2
            else:
                is_leaving_fallback = delta_h < -self.MOVING_AWAY_SIZE_THRESHOLD

            if is_leaving_fallback: 
                final_intent = "MOVING_AWAY"
                debug_str = "[CAM FALLBACK] Bounding box shrinking"
            elif area_ratio > self.INTERACT_THRESH_RATIO:
                final_intent = "WANT_TO_INTERACT"
                debug_str = f"[CAM FALLBACK] Area: {area_ratio*100:.0f}%"
            elif area_ratio > self.CLOSE_THRESH_RATIO:
                final_intent = "CLOSE_PROXIMITY"
                debug_str = f"[CAM FALLBACK] Area: {area_ratio*100:.0f}%"
            else:
                final_intent = "STATIONARY"
                debug_str = "[CAM FALLBACK] Static/Far"

        stable_intent = self.stabilizer.update(final_intent)
        self.current_stable_intent = stable_intent

        self.prev_cam_avg_w = curr_w
        self.prev_cam_avg_h = curr_h

        self.get_logger().info(f"Intent: {stable_intent} | {debug_str}")

        out = String()
        out.data = stable_intent
        self.intent_publisher.publish(out)

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
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
