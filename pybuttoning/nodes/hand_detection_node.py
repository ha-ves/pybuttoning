#!/usr/bin/env python3
"""
Hand detection node using MediaPipe.
Subscribes to camera images and publishes hand landmarks.
No threading needed - this is a separate ROS2 process.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from buttoning_msgs.msg import HandLandmarks
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import time


class HandDetectionNode(Node):
    """ROS2 node for hand detection using MediaPipe."""
    
    def __init__(self):
        super().__init__('hand_detection_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # State
        self.hand_detected = False
        
        # Initialize MediaPipe hands
        try:
            import mediapipe as mp
            self.mp_hands = mp.solutions.hands
            self.hands = self.mp_hands.Hands(
                static_image_mode=False,
                max_num_hands=2,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5
            )
            self.mp_available = True
            self.get_logger().info('MediaPipe initialized successfully')
        except ImportError:
            self.get_logger().warn('MediaPipe not available, hand detection disabled')
            self.mp_available = False
            self.hands = None
        
        # Publishers
        self.hand_pub = self.create_publisher(HandLandmarks, 'hand_landmarks', 10)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        
        self.get_logger().info('Hand detection node initialized')

    def image_callback(self, msg):
        """Process incoming image for hand detection."""
        if not self.mp_available:
            return
            
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self.process_hands(frame)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def process_hands(self, frame):
        """Process frame for hand detection."""
        start_time = time.time()
        
        # Run MediaPipe hand detection
        results = self.hands.process(frame)
        
        end_time = time.time()
        self.processing_time = end_time - start_time
        
        # Create and publish HandLandmarks message
        msg = HandLandmarks()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.processing_time = self.processing_time
        
        if results.multi_hand_landmarks:
            self.hand_detected = True
            msg.hand_detected = True
            
            # Get first hand (or combine multiple hands if needed)
            hand_landmarks = results.multi_hand_landmarks[0]
            
            # Convert landmarks to Point messages
            for landmark in hand_landmarks.landmark:
                point = Point()
                point.x = landmark.x
                point.y = landmark.y
                point.z = landmark.z
                msg.landmarks.append(point)
        else:
            self.hand_detected = False
            msg.hand_detected = False
        
        self.hand_pub.publish(msg)

    def destroy_node(self):
        """Clean up resources."""
        if self.hands:
            self.hands.close()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
