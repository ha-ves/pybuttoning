#!/usr/bin/env python3
"""
Custom RealSense node for pybuttoning package.
Directly interfaces with RealSense camera using pyrealsense2d library.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import os
import sys
import json


class RealSenseNode(Node):
    """ROS2 node for RealSense camera streaming."""
    
    def __init__(self):
        super().__init__('realsense_node')
        
        # Declare parameters
        self.declare_parameter('rs2_dll_path', 'rs2')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('enable_depth', True)
        self.declare_parameter('enable_color', True)
        self.declare_parameter('align_to_color', True)
        self.declare_parameter('config_json', '')
        self.declare_parameter('publish_rate', 30.0)
        
        # Get parameters
        rs2_dll_path = self.get_parameter('rs2_dll_path').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        enable_depth = self.get_parameter('enable_depth').value
        enable_color = self.get_parameter('enable_color').value
        align_to_color = self.get_parameter('align_to_color').value
        config_json = self.get_parameter('config_json').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize RealSense
        self.get_logger().info('Initializing RealSense camera...')
        
        # Add DLL directory for Windows
        if sys.platform == 'win32' and os.path.isdir(rs2_dll_path):
            self.dll_context = os.add_dll_directory(os.path.abspath(rs2_dll_path))
        else:
            self.dll_context = None
        
        try:
            import pyrealsense2 as rs
            self.rs = rs
        except ImportError:
            self.get_logger().error('Failed to import pyrealsense2d. Please install the library.')
            raise
        
        # Create pipeline and config
        self.pipeline = rs.pipeline()
        config = rs.config()
        
        # Configure streams
        if enable_depth:
            config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
            self.get_logger().info(f'Enabled depth stream: {width}x{height} @ {fps}fps')
        
        if enable_color:
            config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
            self.get_logger().info(f'Enabled color stream: {width}x{height} @ {fps}fps')
        
        # Start pipeline
        try:
            profile = self.pipeline.start(config)
            self.get_logger().info('Pipeline started successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to start pipeline: {e}')
            raise
        
        # Setup alignment if requested
        self.align_to_color = align_to_color and enable_depth and enable_color
        if self.align_to_color:
            self.align = rs.align(rs.stream.color)
            self.get_logger().info('Depth alignment to color enabled')
        
        # Apply JSON configuration if provided
        if config_json and os.path.isfile(config_json):
            try:
                device = profile.get_device()
                depth_sensor = device.first_depth_sensor()
                self._apply_json_config(config_json, depth_sensor)
                self.get_logger().info(f'Applied config from {config_json}')
            except Exception as e:
                self.get_logger().warning(f'Failed to apply JSON config: {e}')
        
        # Get camera intrinsics
        self.intrinsics = None
        self.camera_info_msg = None
        if enable_color:
            try:
                color_profile = profile.get_stream(rs.stream.color)
                self.intrinsics = color_profile.as_video_stream_profile().get_intrinsics()
                self.camera_info_msg = self._create_camera_info_msg()
                self.get_logger().info('Camera intrinsics retrieved')
            except Exception as e:
                self.get_logger().warning(f'Failed to get intrinsics: {e}')
        
        # Create publishers
        self.enable_color = enable_color
        self.enable_depth = enable_depth
        
        if enable_color:
            self.color_pub = self.create_publisher(
                Image, '~/color/image_raw', 10)
            self.color_info_pub = self.create_publisher(
                CameraInfo, '~/color/camera_info', 10)
            self.get_logger().info('Color publishers created')
        
        if enable_depth:
            self.depth_pub = self.create_publisher(
                Image, '~/depth/image_rect_raw', 10)
            if self.align_to_color:
                self.aligned_depth_pub = self.create_publisher(
                    Image, '~/aligned_depth_to_color/image_raw', 10)
                self.get_logger().info('Aligned depth publisher created')
        
        # Create timer for frame capture
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.frame_count = 0
        self.get_logger().info(f'RealSense node initialized (publish rate: {publish_rate} Hz)')
    
    def _apply_json_config(self, json_path, sensor):
        """Apply JSON configuration to depth sensor."""
        try:
            with open(json_path, 'r') as f:
                config_data = json.load(f)
            
            # Get the controls
            if 'param-depthunits' in config_data:
                depth_units = config_data['param-depthunits']
                sensor.set_option(self.rs.option.depth_units, depth_units)
            
            if 'controls-depth' in config_data:
                for control in config_data['controls-depth']:
                    name = control.get('name')
                    value = control.get('value')
                    
                    # Map control names to rs.option
                    option_map = {
                        'Depth Units': self.rs.option.depth_units,
                        'Laser Power': self.rs.option.laser_power,
                        'Visual Preset': self.rs.option.visual_preset,
                        'Confidence Threshold': self.rs.option.confidence_threshold,
                    }
                    
                    if name in option_map and sensor.supports(option_map[name]):
                        sensor.set_option(option_map[name], value)
                        self.get_logger().debug(f'Set {name} = {value}')
        
        except Exception as e:
            self.get_logger().warning(f'Error applying JSON config: {e}')
    
    def _create_camera_info_msg(self):
        """Create CameraInfo message from intrinsics."""
        if not self.intrinsics:
            return None
        
        msg = CameraInfo()
        msg.width = self.intrinsics.width
        msg.height = self.intrinsics.height
        msg.distortion_model = 'plumb_bob'
        
        # Intrinsic camera matrix
        msg.k = [
            self.intrinsics.fx, 0.0, self.intrinsics.ppx,
            0.0, self.intrinsics.fy, self.intrinsics.ppy,
            0.0, 0.0, 1.0
        ]
        
        # Distortion coefficients
        msg.d = list(self.intrinsics.coeffs[:5])
        
        # Rectification matrix (identity for unrectified)
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        # Projection matrix
        msg.p = [
            self.intrinsics.fx, 0.0, self.intrinsics.ppx, 0.0,
            0.0, self.intrinsics.fy, self.intrinsics.ppy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        return msg
    
    def timer_callback(self):
        """Capture and publish frames."""
        try:
            # Wait for frames
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            
            # Align frames if requested
            if self.align_to_color:
                frames = self.align.process(frames)
            
            # Get timestamp
            timestamp = self.get_clock().now().to_msg()
            
            # Process and publish color frame
            if self.enable_color:
                color_frame = frames.get_color_frame()
                if color_frame:
                    self._publish_color_frame(color_frame, timestamp)
            
            # Process and publish depth frame
            if self.enable_depth:
                depth_frame = frames.get_depth_frame()
                if depth_frame:
                    self._publish_depth_frame(depth_frame, timestamp)
            
            self.frame_count += 1
            
            # Log every 100 frames
            if self.frame_count % 100 == 0:
                self.get_logger().debug(f'Published {self.frame_count} frames')
        
        except Exception as e:
            self.get_logger().warning(f'Frame capture error: {e}')
    
    def _publish_color_frame(self, frame, timestamp):
        """Publish color frame as ROS Image message."""
        try:
            # Convert to numpy array
            color_image = np.asanyarray(frame.get_data())
            
            # Create ROS message
            header = Header()
            header.stamp = timestamp
            header.frame_id = 'camera_color_optical_frame'
            
            # Convert to ROS Image (RGB8 format)
            img_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='rgb8')
            img_msg.header = header
            
            # Publish
            self.color_pub.publish(img_msg)
            
            # Publish camera info
            if self.camera_info_msg:
                info_msg = self.camera_info_msg
                info_msg.header = header
                self.color_info_pub.publish(info_msg)
        
        except Exception as e:
            self.get_logger().warning(f'Error publishing color frame: {e}')
    
    def _publish_depth_frame(self, frame, timestamp):
        """Publish depth frame as ROS Image message."""
        try:
            # Convert to numpy array
            depth_image = np.asanyarray(frame.get_data())
            
            # Create ROS message
            header = Header()
            header.stamp = timestamp
            header.frame_id = 'camera_depth_optical_frame'
            
            # Convert to ROS Image (16UC1 format)
            img_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
            img_msg.header = header
            
            # Publish
            if self.align_to_color:
                self.aligned_depth_pub.publish(img_msg)
            else:
                self.depth_pub.publish(img_msg)
        
        except Exception as e:
            self.get_logger().warning(f'Error publishing depth frame: {e}')
    
    def destroy_node(self):
        """Cleanup when node is destroyed."""
        try:
            self.get_logger().info('Stopping RealSense pipeline...')
            self.pipeline.stop()
            
            if self.dll_context:
                self.dll_context.close()
            
            self.get_logger().info('RealSense node shutdown complete')
        except Exception as e:
            self.get_logger().error(f'Error during shutdown: {e}')
        
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = RealSenseNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
