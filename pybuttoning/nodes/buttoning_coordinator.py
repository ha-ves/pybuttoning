#!/usr/bin/env python3
"""
Buttoning coordinator node - orchestrates the buttoning task.
"""

import rclpy
from rclpy.node import Node
from buttoning_msgs.msg import Detection
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String, Bool
import numpy as np


class ButtoningCoordinator(Node):
    """ROS2 node for coordinating the buttoning task."""
    
    def __init__(self):
        super().__init__('buttoning_coordinator')
        
        # State machine
        self.state = 'IDLE'
        # States: IDLE, SEARCHING, ALIGNING, APPROACHING, BUTTONING, DONE
        
        # Latest detection data
        self.latest_detection = None
        self.button_track_id = None
        self.buttonhole_track_id = None
        
        # Subscribers
        self.detection_sub = self.create_subscription(
            Detection, 'detections',
            self.detection_callback, 10)
        
        self.left_arm_status_sub = self.create_subscription(
            String, 'left_arm/status',
            self.left_arm_status_callback, 10)
        
        self.right_arm_status_sub = self.create_subscription(
            String, 'right_arm/status',
            self.right_arm_status_callback, 10)
        
        # Publishers
        self.left_target_pub = self.create_publisher(
            Pose, 'left_arm/target_pose', 10)
        
        self.right_target_pub = self.create_publisher(
            Pose, 'right_arm/target_pose', 10)
        
        self.left_gripper_pub = self.create_publisher(
            Point, 'left_arm/gripper_cmd', 10)
        
        self.right_gripper_pub = self.create_publisher(
            Point, 'right_arm/gripper_cmd', 10)
        
        self.state_pub = self.create_publisher(
            String, 'coordinator/state', 10)
        
        # Timer for state machine
        self.timer = self.create_timer(0.1, self.update_state_machine)
        
        self.get_logger().info('Buttoning coordinator initialized')

    def detection_callback(self, msg: Detection):
        """Handle detection message."""
        self.latest_detection = msg

    def left_arm_status_callback(self, msg: String):
        """Handle left arm status updates."""
        self.get_logger().info(f'Left arm: {msg.data}')

    def right_arm_status_callback(self, msg: String):
        """Handle right arm status updates."""
        self.get_logger().info(f'Right arm: {msg.data}')

    def update_state_machine(self):
        """Update state machine."""
        # Publish current state
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)
        
        if self.state == 'IDLE':
            # Wait for start command or automatically start
            if self.latest_detection is not None and len(self.latest_detection.track_ids) > 0:
                self.state = 'SEARCHING'
                self.get_logger().info('Starting buttoning task...')
        
        elif self.state == 'SEARCHING':
            # Find button and buttonhole from Detection message
            if self.latest_detection is None:
                return
                
            button_idx = None
            buttonhole_idx = None
            
            for i, (cls, score, track_id) in enumerate(zip(
                self.latest_detection.classes, 
                self.latest_detection.scores,
                self.latest_detection.track_ids)):
                if cls == 0 and score > 0.7:  # button
                    button_idx = i
                elif cls == 1 and score > 0.7:  # buttonhole
                    buttonhole_idx = i
            
            if button_idx is not None and buttonhole_idx is not None:
                self.button_track_id = self.latest_detection.track_ids[button_idx]
                self.buttonhole_track_id = self.latest_detection.track_ids[buttonhole_idx]
                self.state = 'ALIGNING'
                self.get_logger().info(
                    f'Found button (ID: {self.button_track_id}) and '
                    f'buttonhole (ID: {self.buttonhole_track_id})')
        
        elif self.state == 'ALIGNING':
            # Get current positions from detection
            if self.latest_detection is None:
                return
                
            button_idx = self._get_index_by_track_id(self.button_track_id)
            buttonhole_idx = self._get_index_by_track_id(self.buttonhole_track_id)
            
            if button_idx is not None and buttonhole_idx is not None:
                # Command arms to align with targets
                # TODO: Convert 2D centers to 3D positions using depth
                # For now, just log the detection
                self.get_logger().info('Aligning with detected objects...')
                
                # Move to next state
                self.state = 'APPROACHING'
        
        elif self.state == 'APPROACHING':
            # Approach targets slowly
            self.get_logger().info('Approaching targets...')
            # TODO: Implement careful approach with force feedback
            self.state = 'BUTTONING'
        
        elif self.state == 'BUTTONING':
            # Execute buttoning motion
            self.get_logger().info('Executing buttoning motion...')
            # TODO: Implement precise buttoning sequence
            self.state = 'DONE'
        
        elif self.state == 'DONE':
            self.get_logger().info('Buttoning complete!')
            # Reset or wait for next task
            pass

    def _get_index_by_track_id(self, track_id):
        """Get detection index by track ID."""
        if self.latest_detection is None:
            return None
        try:
            return self.latest_detection.track_ids.index(track_id)
        except ValueError:
            return None


def main(args=None):
    rclpy.init(args=args)
    node = ButtoningCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
