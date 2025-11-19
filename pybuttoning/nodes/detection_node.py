#!/usr/bin/env python3
"""
Detection node for button/buttonhole detection and tracking.
Subscribes to camera image topic and publishes Detection messages.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from buttoning_msgs.msg import Detection, HandLandmarks
from cv_bridge import CvBridge
import torch
import numpy as np
import cv2
import cloudpickle
import os
import time

from pybuttoning.utils.tracking import SortTrack


class DetectionNode(Node):
    """ROS2 node for object detection and tracking."""
    
    def __init__(self):
        super().__init__('detection_node')
        
        # Declare parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('max_detections', 10)
        self.declare_parameter('button_threshold', 0.8)
        self.declare_parameter('buttonhole_threshold', 0.7)
        self.declare_parameter('iou_threshold', 0.01)
        self.declare_parameter('max_age', 10)
        self.declare_parameter('min_hits', 1)
        
        # Get parameters
        model_path = self.get_parameter('model_path').value
        self.max_detections = self.get_parameter('max_detections').value
        self.button_threshold = self.get_parameter('button_threshold').value
        self.buttonhole_threshold = self.get_parameter('buttonhole_threshold').value
        iou_thr = self.get_parameter('iou_threshold').value
        max_age = self.get_parameter('max_age').value
        min_hits = self.get_parameter('min_hits').value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # State
        self.current_frame = None
        self.frame_received = False
        self.hand_detected = False
        self.not_found_counter = 0
        
        # Load detection model
        if model_path:
            self.get_logger().info(f'Loading model from {model_path}...')
            self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
            self.model = self._load_model(model_path)
        else:
            self.get_logger().warn('No model_path specified, detection will not work!')
            self.model = None
        
        # Initialize SORT tracker
        self.tracker = SortTrack(
            iou_thr=iou_thr, 
            max_age=max_age, 
            min_hits=min_hits,
            guard=[2, 3]  # [button_guard, buttonhole_guard]
        )
        
        # Publishers
        self.detection_pub = self.create_publisher(Detection, 'detections', 10)
        self.debug_image_pub = self.create_publisher(Image, 'debug_image', 10)
        
        # Subscribers - topic name can be remapped in launch file
        self.image_sub = self.create_subscription(
            Image, 'image_raw', self.image_callback, 10)
        
        self.hand_sub = self.create_subscription(
            HandLandmarks, 'hand_landmarks', self.hand_landmarks_callback, 10)
        
        # Timer for main loop (30 Hz)
        self.timer = self.create_timer(0.033, self.main_loop)
        
        self.last_time = time.time()
        self.get_logger().info('Detection node initialized')

    def _load_model(self, model_dir):
        """Load Detectron2 model."""
        from detectron2.config import LazyConfig, instantiate
        from detectron2.checkpoint import DetectionCheckpointer
        
        config_path = os.path.join(model_dir, 'config.yaml.pkl')
        with open(config_path, 'rb') as f:
            cfg = cloudpickle.load(f)
        
        cfg.model.roi_heads.box_predictor.test_score_thresh = 0.1
        cfg.model.roi_heads.box_predictor.test_nms_thresh = 0.1
        
        # Disable mask prediction for speed
        cfg.model.roi_heads.mask_in_features = None
        cfg.model.roi_heads.mask_head = None
        cfg.model.roi_heads.mask_pooler = None
        
        model = instantiate(cfg.model)
        model.to(self.device)
        
        checkpointer = DetectionCheckpointer(model)
        model_path = os.path.join(model_dir, 'model_best_segm_AP-buttonhole.pth')
        checkpointer.load(model_path)
        model.eval()
        
        return model

    def image_callback(self, msg):
        """Store latest image frame."""
        try:
            cv_ptr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self.current_frame = cv_ptr
            self.frame_received = True
        except Exception as e:
            self.get_logger().error(f'cv_bridge exception: {e}')

    def hand_landmarks_callback(self, msg):
        """Update hand detection status."""
        self.hand_detected = msg.hand_detected

    def main_loop(self):
        """Main processing loop."""
        if not self.frame_received:
            self.get_logger().warn(
                "Waiting for images on 'image_raw' topic...",
                throttle_duration_sec=5.0)
            return
        
        self.process_frame()

    def process_frame(self):
        """Process current frame and publish detections."""
        if self.current_frame is None or self.model is None:
            return
        
        # Calculate dt
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Convert to tensor for model
        input_tensor = torch.from_numpy(self.current_frame).to(self.device)
        input_tensor = input_tensor.to(torch.float).permute(2, 0, 1)[[2, 1, 0], :, :]
        
        # Run detection
        with torch.no_grad():
            outputs = self.model([{"image": input_tensor}])[0]
        
        instances = outputs["instances"]
        boxes = instances.pred_boxes.tensor  # [N, 4]
        scores = instances.scores  # [N]
        classes = instances.pred_classes  # [N]
        
        # Apply per-class thresholds
        valid_mask = torch.zeros(len(boxes), dtype=torch.bool, device=self.device)
        for cls_id in [0, 1]:  # button, buttonhole
            cls_mask = classes == cls_id
            if cls_id == 0:  # button
                valid_mask |= (cls_mask & (scores > self.button_threshold))
            else:  # buttonhole
                valid_mask |= (cls_mask & (scores > self.buttonhole_threshold))
        
        boxes = boxes[valid_mask]
        classes = classes[valid_mask]
        scores = scores[valid_mask]
        
        # Update tracker
        if len(boxes) > 0:
            preds, mappings, _ = self.tracker.update(boxes, classes, dt)
        else:
            mappings = torch.empty((0, 2), dtype=torch.long)
        
        # Create Detection message
        msg = Detection()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'
        
        debug_image = self.current_frame.copy()
        
        # Flatten arrays for the Detection message format
        boxes_flat = []
        classes_flat = []
        scores_flat = []
        centers_flat = []
        track_ids_flat = []
        
        for det_idx, track_id in mappings:
            det_idx = det_idx.item()
            track_id = track_id.item()
            
            box = boxes[det_idx]
            cls = classes[det_idx].item()
            score = scores[det_idx].item()
            
            # Get box center
            cx = int((box[0] + box[2]) / 2)
            cy = int((box[1] + box[3]) / 2)
            
            # Append to flat arrays
            boxes_flat.extend([int(box[0]), int(box[1]), int(box[2]), int(box[3])])
            classes_flat.append(cls)
            scores_flat.append(score)
            centers_flat.extend([cx, cy])
            track_ids_flat.append(track_id)
            
            # Draw on debug image
            color = (0, 255, 0) if cls == 0 else (255, 0, 0)
            class_name = 'button' if cls == 0 else 'buttonhole'
            cv2.rectangle(debug_image, 
                         (int(box[0]), int(box[1])), 
                         (int(box[2]), int(box[3])), 
                         color, 2)
            cv2.putText(debug_image, f'{class_name}:{track_id}', 
                       (int(box[0]), int(box[1]) - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Set message fields
        msg.boxes = boxes_flat
        msg.classes = classes_flat
        msg.scores = scores_flat
        msg.centers = centers_flat
        msg.track_ids = track_ids_flat
        
        # Publish
        self.detection_pub.publish(msg)
        
        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='rgb8')
        debug_msg.header = msg.header
        self.debug_image_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
