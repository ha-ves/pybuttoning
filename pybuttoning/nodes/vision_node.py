#!/usr/bin/env python3
"""
Vision node for button/buttonhole detection and tracking.
Subscribes to RealSense camera topics and publishes detected objects.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from buttoning_msgs.msg import Detection
from cv_bridge import CvBridge
import torch
import numpy as np
import cv2
import cloudpickle
import os

from pybuttoning.utils.tracking import SortTrack


class VisionNode(Node):
    """ROS2 node for vision processing."""
    
    def __init__(self):
        super().__init__('vision_node')
        
        # Declare parameters
        self.declare_parameter('model_dir', '')
        self.declare_parameter('score_threshold', 0.1)
        self.declare_parameter('nms_threshold', 0.1)
        self.declare_parameter('camera_config', '')
        self.declare_parameter('use_tracking', True)
        self.declare_parameter('iou_threshold', 0.01)
        self.declare_parameter('max_age', 10)
        
        # Get parameters
        model_dir = self.get_parameter('model_dir').value
        score_thresh = self.get_parameter('score_threshold').value
        nms_thresh = self.get_parameter('nms_threshold').value
        camera_config = self.get_parameter('camera_config').value
        use_tracking = self.get_parameter('use_tracking').value
        iou_thr = self.get_parameter('iou_threshold').value
        max_age = self.get_parameter('max_age').value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Subscribe to RealSense camera topics
        self.color_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', 
            self.color_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self.camera_info_callback, 10)
        
        # Store latest frames
        self.latest_color = None
        self.latest_depth = None
        self.camera_info = None
        
        # Load detection model
        self.get_logger().info(f'Loading model from {model_dir}...')
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = self._load_model(model_dir, score_thresh, nms_thresh)
        
        # Initialize tracker
        self.use_tracking = use_tracking
        if use_tracking:
            self.tracker = SortTrack(
                iou_thr=iou_thr, 
                max_age=max_age, 
                min_hits=1,
                guard=[2, 3]  # [button_guard, buttonhole_guard]
            )
        
        # Publishers
        self.detection_pub = self.create_publisher(
            Detection, 'detections', 10)
        self.debug_image_pub = self.create_publisher(
            Image, 'debug_image', 10)
        
        # Timer for processing
        self.timer = self.create_timer(0.033, self.process_frame)  # ~30 Hz
        
        self.last_time = self.get_clock().now()
        self.get_logger().info('Vision node initialized')

    def color_callback(self, msg):
        """Store latest color frame."""
        self.latest_color = msg
    
    def depth_callback(self, msg):
        """Store latest depth frame."""
        self.latest_depth = msg
    
    def camera_info_callback(self, msg):
        """Store camera info."""
        self.camera_info = msg

    def _load_model(self, model_dir, score_thresh, nms_thresh):
        """Load Detectron2 model."""
        from detectron2.config import LazyConfig, instantiate
        from detectron2.checkpoint import DetectionCheckpointer
        
        config_path = os.path.join(model_dir, 'config.yaml.pkl')
        with open(config_path, 'rb') as f:
            cfg = cloudpickle.load(f)
        
        cfg.model.roi_heads.box_predictor.test_score_thresh = score_thresh
        cfg.model.roi_heads.box_predictor.test_nms_thresh = nms_thresh
        
        # Optionally disable mask prediction for speed
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

    def process_frame(self):
        """Process camera frame and publish detections."""
        # Check if we have frames
        if self.latest_color is None or self.latest_depth is None:
            return
        
        # Convert ROS images to OpenCV
        try:
            color_frame = self.bridge.imgmsg_to_cv2(self.latest_color, desired_encoding='rgb8')
            depth_frame = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Failed to convert images: {e}')
            return
        
        # Calculate dt
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Convert to tensor for model
        input_tensor = torch.from_numpy(color_frame).to(self.device)
        input_tensor = input_tensor.to(torch.float).permute(2, 0, 1)[[2, 1, 0], :, :]
        
        # Run detection
        with torch.no_grad():
            outputs = self.model([{"image": input_tensor}])[0]
        
        instances = outputs["instances"]
        boxes = instances.pred_boxes.tensor  # [N, 4]
        scores = instances.scores  # [N]
        classes = instances.pred_classes  # [N]
        
        # Filter detections if needed
        valid = scores > 0.1
        boxes = boxes[valid]
        classes = classes[valid]
        scores = scores[valid]
        
        # Update tracker
        if self.use_tracking and len(boxes) > 0:
            preds, mappings, _ = self.tracker.update(boxes, classes, dt)
        else:
            mappings = torch.tensor([[i, i] for i in range(len(boxes))], dtype=torch.long)
        
        # Create Detection message using buttoning_msgs format
        msg = Detection()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'
        
        debug_image = color_frame.copy()
        
        # Flatten arrays for the Detection message format
        boxes_flat = []
        classes_flat = []
        scores_flat = []
        centers_flat = []
        track_ids_flat = []
        
        for i, (det_idx, track_id) in enumerate(mappings):
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
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
