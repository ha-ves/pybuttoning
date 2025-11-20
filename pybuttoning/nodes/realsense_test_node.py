#!/usr/bin/env python3
"""
Test script for RealSense node.
Subscribes to camera topics and displays frame info.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np


class RealSenseTestNode(Node):
    """Test node for RealSense camera."""
    
    def __init__(self):
        super().__init__('realsense_test_node')
        
        self.bridge = CvBridge()
        
        # Subscribe to topics
        self.color_sub = self.create_subscription(
            Image,
            '/camera/realsense_node/color/image_raw',
            self.color_callback,
            10)
        
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/realsense_node/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10)
        
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/realsense_node/color/camera_info',
            self.info_callback,
            10)
        
        self.color_count = 0
        self.depth_count = 0
        self.info_received = False
        
        # Timer for status
        self.timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info('RealSense test node started')
    
    def color_callback(self, msg):
        """Handle color frame."""
        self.color_count += 1
        if self.color_count == 1:
            self.get_logger().info(
                f'First color frame: {msg.width}x{msg.height}, '
                f'encoding: {msg.encoding}')
    
    def depth_callback(self, msg):
        """Handle depth frame."""
        self.depth_count += 1
        if self.depth_count == 1:
            self.get_logger().info(
                f'First depth frame: {msg.width}x{msg.height}, '
                f'encoding: {msg.encoding}')
    
    def info_callback(self, msg):
        """Handle camera info."""
        if not self.info_received:
            self.info_received = True
            self.get_logger().info(
                f'Camera info: {msg.width}x{msg.height}, '
                f'fx={msg.k[0]:.2f}, fy={msg.k[4]:.2f}')
    
    def print_status(self):
        """Print status periodically."""
        self.get_logger().info(
            f'Status - Color frames: {self.color_count}, '
            f'Depth frames: {self.depth_count}, '
            f'Camera info: {"received" if self.info_received else "not received"}')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = RealSenseTestNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
