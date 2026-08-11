import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from vision_msgs.msg import Point2D, BoundingBox2D

class ThermalPoseSimple(Node):
    def __init__(self):
        super().__init__('thermal_pose_simple')
        try:
            from face_tracker.hailo_yolov8_pose import HailoYolov8Pose
        except ImportError:
            from hailo_yolov8_pose import HailoYolov8Pose
            
        self.model = HailoYolov8Pose('/home/docker_user/ros2_ws_head/src/face_tracker/face_tracker/yolov8s_pose.hef')
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image,
            '/hospibot/image_raw',
            self.image_callback,
            10
        )
        self.publisher_ = self.create_publisher(Image, '/hospibot/image_pose_annotated', 10)
        self.bb_pub = self.create_publisher(BoundingBox2D, '/hospibot/pose_bbox', 10)
        self.tight_bb_pub = self.create_publisher(BoundingBox2D, '/hospibot/pose_bbox_tight', 10)
        self.mask_pub = self.create_publisher(Image, '/hospibot/pose_mask', 10)
        self.eye_center_pub = self.create_publisher(Point2D, '/face_tracker/eye_center', 10)
        
        self.get_logger().info('Simple Pose Node started. (No heuristic logic or rotations)')
        
        # Ultralytics-style COCO skeleton palette
        # Colors in BGR
        self.kpt_colors = [
            (0, 255, 0),   # 0: nose
            (0, 255, 0),   # 1: left_eye
            (0, 255, 0),   # 2: right_eye
            (0, 255, 0),   # 3: left_ear
            (0, 255, 0),   # 4: right_ear
            (0, 165, 255), # 5: left_shoulder
            (0, 165, 255), # 6: right_shoulder
            (0, 255, 255), # 7: left_elbow
            (0, 255, 255), # 8: right_elbow
            (0, 0, 255),   # 9: left_wrist
            (0, 0, 255),   # 10: right_wrist
            (255, 255, 0), # 11: left_hip
            (255, 255, 0), # 12: right_hip
            (255, 0, 255), # 13: left_knee
            (255, 0, 255), # 14: right_knee
            (255, 0, 0),   # 15: left_ankle
            (255, 0, 0)    # 16: right_ankle
        ]
        
        self.skeleton = [
            (15, 13), (13, 11), (16, 14), (14, 12), (11, 12), 
            (5, 11), (6, 12), (5, 6), (5, 7), (6, 8), (7, 9), 
            (8, 10), (1, 2), (0, 1), (0, 2), (1, 3), (2, 4), 
            (3, 5), (4, 6)
        ]
        
        self.limb_colors = [
            (255, 0, 0), (255, 0, 255), (255, 0, 0), (255, 0, 255), (255, 255, 0),
            (255, 255, 0), (255, 255, 0), (0, 165, 255), (0, 255, 255), (0, 255, 255), (0, 0, 255),
            (0, 0, 255), (0, 255, 0), (0, 255, 0), (0, 255, 0), (0, 255, 0), (0, 255, 0),
            (0, 255, 0), (0, 255, 0)
        ]

    def publish_eye_center(self, kpts_np, kpts_conf):
        # 1 = L-Eye, 2 = R-Eye
        if len(kpts_np) > 2:
            left_eye = kpts_np[1]
            right_eye = kpts_np[2]
            
            # Check confidence for both eyes
            if kpts_conf[1] > 0.5 and kpts_conf[2] > 0.5:
                eye_center_x = (left_eye[0] + right_eye[0]) / 2.0
                eye_center_y = (left_eye[1] + right_eye[1]) / 2.0
                
                eye_msg = Point2D()
                eye_msg.x = float(eye_center_x)
                eye_msg.y = float(eye_center_y)
                self.eye_center_pub.publish(eye_msg)
            # If only left eye is visible
            elif kpts_conf[1] > 0.5:
                eye_msg = Point2D()
                eye_msg.x = float(left_eye[0])
                eye_msg.y = float(left_eye[1])
                self.eye_center_pub.publish(eye_msg)
            # If only right eye is visible
            elif kpts_conf[2] > 0.5:
                eye_msg = Point2D()
                eye_msg.x = float(right_eye[0])
                eye_msg.y = float(right_eye[1])
                self.eye_center_pub.publish(eye_msg)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if cv_image.dtype == np.uint16 or cv_image.dtype == np.int16:
                cv_image = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            if len(cv_image.shape) == 2 or cv_image.shape[2] == 1:
                base_img = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            else:
                base_img = cv_image
                
            orig_h, orig_w = base_img.shape[:2]
            final_output = base_img.copy()
            
            # Simple direct inference (no rotations)
            results = self.model(base_img, conf=0.6, verbose=False)
            
            if len(results) > 0:
                bboxes = []
                scores = []
                for det in results:
                    x1, y1, x2, y2 = det['bbox']
                    bboxes.append([x1, y1, x2 - x1, y2 - y1])
                    scores.append(det['score'])
                    
                indices = cv2.dnn.NMSBoxes(bboxes, scores, 0.6, 0.4)
                
                if len(indices) > 0:
                    # Pick the best detection for tracking
                    best_idx = indices.flatten()[0]
                    best_det = results[best_idx]
                    
                    x1, y1, x2, y2 = best_det['bbox']
                    bb_msg = BoundingBox2D()
                    bb_msg.center.position.x = float((x1 + x2) / 2.0)
                    bb_msg.center.position.y = float((y1 + y2) / 2.0)
                    bb_msg.center.theta = 0.0
                    bb_msg.size_x = float(x2 - x1)
                    bb_msg.size_y = float(y2 - y1)
                    self.bb_pub.publish(bb_msg)
                    
                    if 'tight_bbox' in best_det:
                        tx1, ty1, tx2, ty2 = best_det['tight_bbox']
                        tight_bb_msg = BoundingBox2D()
                        tight_bb_msg.center.position.x = float((tx1 + tx2) / 2.0)
                        tight_bb_msg.center.position.y = float((ty1 + ty2) / 2.0)
                        tight_bb_msg.center.theta = 0.0
                        tight_bb_msg.size_x = float(tx2 - tx1)
                        tight_bb_msg.size_y = float(ty2 - ty1)
                        self.tight_bb_pub.publish(tight_bb_msg)
                        
                        # Draw tight bounding box in green
                        cv2.rectangle(final_output, (int(tx1), int(ty1)), (int(tx2), int(ty2)), (0, 255, 0), 2)
                    
                    # Create a blank mask for the best detection
                    mask_img = np.zeros((orig_h, orig_w), dtype=np.uint8)
                    
                    for idx in indices.flatten():
                        det = results[idx]
                        
                        # Get keypoints
                        kpts_list = det['keypoints']
                        kpts_np = np.array([[k[0], k[1]] for k in kpts_list])
                        kpts_conf = np.array([k[2] for k in kpts_list])
                        
                        # Only publish eye center for the main tracked person
                        if idx == best_idx:
                            self.publish_eye_center(kpts_np, kpts_conf)
                        
                        # Draw clean skeleton
                        # 1. Draw Limbs
                        for i, pair in enumerate(self.skeleton):
                            pt1_idx, pt2_idx = pair
                            if kpts_conf[pt1_idx] > 0.5 and kpts_conf[pt2_idx] > 0.5:
                                pt1 = (int(kpts_np[pt1_idx][0]), int(kpts_np[pt1_idx][1]))
                                pt2 = (int(kpts_np[pt2_idx][0]), int(kpts_np[pt2_idx][1]))
                                color = self.limb_colors[i]
                                cv2.line(final_output, pt1, pt2, color, 2, cv2.LINE_AA)
                                
                                # Draw thick limb on mask if it's the tracked person
                                if idx == best_idx:
                                    cv2.line(mask_img, pt1, pt2, 255, 35, cv2.LINE_AA)
                        
                        # 2. Draw Keypoints
                        for i, pt in enumerate(kpts_np):
                            if kpts_conf[i] > 0.5:
                                color = self.kpt_colors[i]
                                # Draw filled circle with outline
                                cv2.circle(final_output, (int(pt[0]), int(pt[1])), 4, color, -1, cv2.LINE_AA)
                                cv2.circle(final_output, (int(pt[0]), int(pt[1])), 4, (255, 255, 255), 1, cv2.LINE_AA)
                                
                                # Draw thick circle on mask if it's the tracked person
                                if idx == best_idx:
                                    # Make head slightly larger for better coverage
                                    radius = 30 if i <= 4 else 20
                                    cv2.circle(mask_img, (int(pt[0]), int(pt[1])), radius, 255, -1, cv2.LINE_AA)
                                
                        # 3. Draw Bounding Box
                        bx1, by1, bx2, by2 = det['bbox']
                        cv2.rectangle(final_output, (int(bx1), int(by1)), (int(bx2), int(by2)), (255, 0, 0), 2)

                    mask_msg = self.bridge.cv2_to_imgmsg(mask_img, encoding="mono8")
                    mask_msg.header = msg.header
                    self.mask_pub.publish(mask_msg)

            annotated_msg = self.bridge.cv2_to_imgmsg(final_output, encoding="bgr8")
            annotated_msg.header = msg.header
            self.publisher_.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ThermalPoseSimple()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
