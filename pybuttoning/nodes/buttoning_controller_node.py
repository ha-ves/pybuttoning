#!/usr/bin/env python3
"""
Buttoning controller node - implements the state machine for buttoning task.
Subscribes to detections and hand landmarks, publishes robot commands.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from buttoning_msgs.msg import Detection, HandLandmarks, RobotCommand
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import numpy as np
import torch
import time
import math


class ButtoningControllerNode(Node):
    """ROS2 node for buttoning task coordination and state machine."""
    
    def __init__(self):
        super().__init__('buttoning_controller_node')
        
        # Declare parameters
        self.declare_parameter('angle_deg', 8.0)  # max angle difference for finger pointing
        self.declare_parameter('distance_thresh', 0.05)  # distance threshold in meters
        self.declare_parameter('choose_time', 1.0)  # time to confirm selection
        
        # Get parameters
        angle_deg = self.get_parameter('angle_deg').value
        self.similarity_thresh = math.cos(math.radians(angle_deg))
        self.distance_thresh = self.get_parameter('distance_thresh').value
        self.choose_time = self.get_parameter('choose_time').value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # State variables
        self.step = 1  # 1=choose button, 2=choose buttonhole, 3=execute buttoning
        self.btn_idx = -1
        self.btnhl_idx = -1
        self.stable_idx = -1
        self.stable_since = None
        
        # Latest messages
        self.latest_detections = None
        self.latest_hand_landmarks = None
        self.latest_depth_image = None
        self.camera_intrinsics = None
        
        # Motion state
        self.motion_in_progress = False
        
        # Publishers
        self.command_pub = self.create_publisher(RobotCommand, 'robot_command', 10)
        self.debug_image_pub = self.create_publisher(Image, 'controller_debug_image', 10)
        
        # Subscribers
        self.detection_sub = self.create_subscription(
            Detection, 'detections', self.detection_callback, 10)
        
        self.hand_sub = self.create_subscription(
            HandLandmarks, 'hand_landmarks', self.hand_landmarks_callback, 10)
        
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', 
            self.depth_callback, 10)
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info',
            self.camera_info_callback, 10)
        
        self.motion_complete_sub = self.create_subscription(
            Bool, 'motion_complete', self.motion_complete_callback, 10)
        
        self.get_logger().info('Buttoning controller node initialized')

    def detection_callback(self, msg: Detection):
        """Store latest detections."""
        self.latest_detections = msg
        self.process_state_machine()

    def hand_landmarks_callback(self, msg: HandLandmarks):
        """Store latest hand landmarks."""
        self.latest_hand_landmarks = msg

    def depth_callback(self, msg: Image):
        """Store latest depth image."""
        try:
            # Depth is uint16, distance in mm
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

    def camera_info_callback(self, msg: CameraInfo):
        """Store camera intrinsics."""
        if self.camera_intrinsics is None:
            self.camera_intrinsics = {
                'fx': msg.k[0],
                'fy': msg.k[4],
                'ppx': msg.k[2],
                'ppy': msg.k[5],
                'width': msg.width,
                'height': msg.height
            }
            self.get_logger().info(f'Camera intrinsics received: {self.camera_intrinsics}')

    def motion_complete_callback(self, msg: Bool):
        """Handle motion completion."""
        self.motion_in_progress = False
        if msg.data:
            self.get_logger().info(f'Motion completed successfully')
        else:
            self.get_logger().warn(f'Motion failed')

    def process_state_machine(self):
        """Main state machine logic."""
        if self.latest_detections is None:
            return
        
        # Parse detections
        num_detections = len(self.latest_detections.track_ids)
        if num_detections == 0:
            return
        
        # Step 1: Choose button
        if self.step == 1:
            self.process_finger_pointing(class_id=0)  # button
            if self.btn_idx >= 0:
                # Check if button still tracked
                if not self._is_tracked(self.btn_idx):
                    self.get_logger().warn('Button lost, resetting selection')
                    self.btn_idx = -1
                    self.stable_idx = -1
        
        # Step 2: Choose buttonhole
        elif self.step == 2:
            # Check if button still visible
            if self.btn_idx >= 0 and not self._is_tracked(self.btn_idx):
                self.get_logger().warn('Button lost, returning to step 1')
                self.btn_idx = -1
                self.step = 1
                return
            
            self.process_finger_pointing(class_id=1)  # buttonhole
            if self.btnhl_idx >= 0:
                # Check if buttonhole still tracked
                if not self._is_tracked(self.btnhl_idx):
                    self.get_logger().warn('Buttonhole lost, resetting selection')
                    self.btnhl_idx = -1
                    self.stable_idx = -1
                # Both selected, move to execution
                elif self._is_tracked(self.btnhl_idx):
                    self.get_logger().info('Both button and buttonhole selected, starting execution')
                    self.step = 3
                    self.send_command('MOVE_TO_HOME')
        
        # Step 3: Execute buttoning
        elif self.step == 3:
            # Check if targets still visible
            if self.btn_idx >= 0 and not self._is_tracked(self.btn_idx):
                self.get_logger().warn('Button lost during execution, aborting')
                self.send_command('STOP')
                self.btn_idx = -1
                self.step = 1
                return
            
            if self.btnhl_idx >= 0 and not self._is_tracked(self.btnhl_idx):
                self.get_logger().warn('Buttonhole lost during execution, aborting')
                self.send_command('STOP')
                self.btnhl_idx = -1
                self.step = 2
                return
            
            # Execution is handled by sending commands to arm controller
            # State progression happens via motion_complete_callback
            if not self.motion_in_progress:
                self.execute_buttoning_sequence()

    def process_finger_pointing(self, class_id):
        """Process finger pointing for object selection."""
        if self.latest_hand_landmarks is None or not self.latest_hand_landmarks.hand_detected:
            return
        
        if self.latest_depth_image is None or self.camera_intrinsics is None:
            return
        
        # Get hand landmarks (MediaPipe format)
        landmarks = self.latest_hand_landmarks.landmarks
        if len(landmarks) < 9:
            return
        
        h, w = self.camera_intrinsics['height'], self.camera_intrinsics['width']
        
        # Index finger base knuckle (landmark 5)
        ind_kn_x = int(landmarks[5].x * w)
        ind_kn_y = int(landmarks[5].y * h)
        if ind_kn_x < 0 or ind_kn_x >= w or ind_kn_y < 0 or ind_kn_y >= h:
            return
        
        # Index fingertip (landmark 8)
        ind_tip_x = int(landmarks[8].x * w)
        ind_tip_y = int(landmarks[8].y * h)
        if ind_tip_x < 0 or ind_tip_x >= w or ind_tip_y < 0 or ind_tip_y >= h:
            return
        
        # Get 3D points
        A = self._deproject_pixel(ind_kn_x, ind_kn_y)  # knuckle
        B = self._deproject_pixel(ind_tip_x, ind_tip_y)  # tip
        if A is None or B is None:
            return
        
        # Get candidate objects of the right class
        candidates = []
        for i in range(len(self.latest_detections.track_ids)):
            if self.latest_detections.classes[i] == class_id:
                track_id = self.latest_detections.track_ids[i]
                cx = self.latest_detections.centers[i * 2]
                cy = self.latest_detections.centers[i * 2 + 1]
                
                # Get 3D point
                O = self._deproject_pixel(cx, cy)
                if O is not None:
                    candidates.append((track_id, O))
        
        if len(candidates) == 0:
            return
        
        # Vector A→B (finger direction)
        vec_AB = B - A
        unit_AB = vec_AB / np.linalg.norm(vec_AB)
        
        # Check each candidate
        valid_candidates = []
        for track_id, O in candidates:
            vec_AO = O - A
            distance_to_B = np.linalg.norm(O - B)
            
            # Normalize and compute cosine similarity
            unit_AO = vec_AO / np.linalg.norm(vec_AO)
            cos_sim = np.dot(unit_AO, unit_AB)
            
            # Check thresholds
            if cos_sim >= self.similarity_thresh and distance_to_B <= self.distance_thresh:
                valid_candidates.append((track_id, cos_sim, distance_to_B))
        
        if len(valid_candidates) == 0:
            return
        
        # Sort by similarity (desc) then distance (asc)
        valid_candidates.sort(key=lambda x: (-x[1], x[2]))
        pointed_idx = valid_candidates[0][0]
        
        # Stability check
        if pointed_idx != self.stable_idx:
            # New selection → start timer
            self.stable_idx = pointed_idx
            self.stable_since = time.time()
            self.get_logger().info(f'Pointing at {"button" if class_id == 0 else "buttonhole"} {pointed_idx}')
        elif time.time() - self.stable_since >= self.choose_time:
            # Same idx for ≥choose_time → confirmed
            if self.step == 1:
                self.btn_idx = pointed_idx
                self.get_logger().info(f'Button {pointed_idx} selected!')
                self.step = 2
                self.stable_idx = -1
            elif self.step == 2:
                self.btnhl_idx = pointed_idx
                self.get_logger().info(f'Buttonhole {pointed_idx} selected!')
                self.stable_idx = -1

    def execute_buttoning_sequence(self):
        """Execute the buttoning manipulation sequence."""
        # This is simplified - full implementation needs detailed trajectory planning
        # In the notebook, this is handled by multiple arm_steps
        
        if self.btn_idx < 0 or self.btnhl_idx < 0:
            return
        
        # Send grasp buttonhole command
        self.send_command('GRASP_BUTTONHOLE', target_id=self.btnhl_idx)
        
        # In full implementation, this would progress through:
        # 1. Grasp buttonhole
        # 2. Grasp button
        # 3. Insert button into buttonhole
        # 4. Release and return home

    def send_command(self, command_type, target_id=-1):
        """Send robot command."""
        msg = RobotCommand()
        msg.command_type = command_type
        msg.target_id = target_id
        
        self.command_pub.publish(msg)
        self.motion_in_progress = True
        self.get_logger().info(f'Sent command: {command_type} (target_id={target_id})')

    def _is_tracked(self, track_id):
        """Check if track_id is in latest detections."""
        if self.latest_detections is None:
            return False
        return track_id in self.latest_detections.track_ids

    def _deproject_pixel(self, x, y):
        """Deproject pixel to 3D point using depth."""
        if self.latest_depth_image is None or self.camera_intrinsics is None:
            return None
        
        # Get depth value (in mm)
        depth_mm = self.latest_depth_image[y, x]
        if depth_mm == 0:
            return None
        
        depth_m = depth_mm / 1000.0
        
        # Deproject using pinhole camera model
        fx = self.camera_intrinsics['fx']
        fy = self.camera_intrinsics['fy']
        ppx = self.camera_intrinsics['ppx']
        ppy = self.camera_intrinsics['ppy']
        
        point_3d = np.array([
            (x - ppx) * depth_m / fx,
            (y - ppy) * depth_m / fy,
            depth_m
        ])
        
        return point_3d


def main(args=None):
    rclpy.init(args=args)
    node = ButtoningControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
