#!/usr/bin/env python3
"""
Arm controller node for controlling both Kinova arms.
Subscribes to detections and robot commands, publishes arm status.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from buttoning_msgs.msg import Detection, RobotCommand
import numpy as np
import time

from pybuttoning.interfaces.kinova_arm import KinovaArmInterface, DeviceConnection


class ArmControllerNode(Node):
    """ROS2 node for dual-arm control based on detections and commands."""
    
    def __init__(self):
        super().__init__('arm_controller_node')
        
        # Declare parameters
        self.declare_parameter('left_arm_ip', '192.168.1.10')
        self.declare_parameter('right_arm_ip', '192.168.1.11')
        self.declare_parameter('use_simulation', False)
        
        # Get parameters
        left_ip = self.get_parameter('left_arm_ip').value
        right_ip = self.get_parameter('right_arm_ip').value
        use_sim = self.get_parameter('use_simulation').value
        
        # State
        self.latest_detections = None
        self.motion_in_progress = False
        
        # Initialize arm interfaces
        if not use_sim:
            try:
                self.get_logger().info(f'Connecting to left arm at {left_ip}...')
                self.left_arm = KinovaArmInterface(left_ip)
                
                self.get_logger().info(f'Connecting to right arm at {right_ip}...')
                self.right_arm = KinovaArmInterface(right_ip)
                
                self.arms_connected = True
                self.get_logger().info('Arms connected successfully')
            except Exception as e:
                self.get_logger().error(f'Failed to connect to arms: {e}')
                self.arms_connected = False
                self.left_arm = None
                self.right_arm = None
        else:
            self.get_logger().info('Running in simulation mode (no real arms)')
            self.arms_connected = False
            self.left_arm = None
            self.right_arm = None
        
        # Publishers
        self.left_pose_pub = self.create_publisher(Pose, 'left_arm/pose', 10)
        self.right_pose_pub = self.create_publisher(Pose, 'right_arm/pose', 10)
        self.motion_complete_pub = self.create_publisher(Bool, 'motion_complete', 10)
        
        # Subscribers
        self.detection_sub = self.create_subscription(
            Detection, 'detections', self.detection_callback, 10)
        
        self.command_sub = self.create_subscription(
            RobotCommand, 'robot_command', self.command_callback, 10)
        
        # Timer for publishing current poses (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_current_poses)
        
        self.get_logger().info('Arm controller node initialized')

    def detection_callback(self, msg: Detection):
        """Store latest detections for processing."""
        self.latest_detections = msg

    def command_callback(self, msg: RobotCommand):
        """Process robot command and execute motion."""
        if self.motion_in_progress:
            self.get_logger().warn('Motion already in progress, ignoring command')
            return
        
        self.motion_in_progress = True
        
        # Process command based on type
        cmd_type = msg.command_type
        self.get_logger().info(f'Received command: {cmd_type}')
        
        try:
            if cmd_type == 'MOVE_TO_HOME':
                self.move_to_home()
            elif cmd_type == 'GRASP_BUTTON':
                self.grasp_button(msg)
            elif cmd_type == 'GRASP_BUTTONHOLE':
                self.grasp_buttonhole(msg)
            elif cmd_type == 'INSERT_BUTTON':
                self.insert_button(msg)
            elif cmd_type == 'RELEASE':
                self.release_all()
            else:
                self.get_logger().warn(f'Unknown command type: {cmd_type}')
            
            # Signal completion
            complete_msg = Bool()
            complete_msg.data = True
            self.motion_complete_pub.publish(complete_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error executing command: {e}')
            complete_msg = Bool()
            complete_msg.data = False
            self.motion_complete_pub.publish(complete_msg)
        
        finally:
            self.motion_in_progress = False

    def move_to_home(self):
        """Move both arms to home position."""
        self.get_logger().info('Moving to home position')
        
        if self.arms_connected:
            # Define home joint angles (in degrees)
            home_angles_left = np.array([0, 15, 180, -130, 0, 55, 90])
            home_angles_right = np.array([0, 15, 180, -130, 0, 55, 90])
            
            # Move arms
            self.left_arm.move_to_joint_angles(home_angles_left)
            self.right_arm.move_to_joint_angles(home_angles_right)
        else:
            # Simulate delay
            time.sleep(2.0)

    def grasp_button(self, msg: RobotCommand):
        """Grasp button with right arm."""
        self.get_logger().info(f'Grasping button at target_id: {msg.target_id}')
        
        if self.arms_connected and self.latest_detections:
            # Find button in detections
            button_center = self._find_detection_center(msg.target_id, class_id=0)
            if button_center is not None:
                # Convert pixel coordinates to 3D position (requires depth + calibration)
                # This is simplified - real implementation needs depth lookup
                target_pos = np.array([button_center[0] / 1000.0, 
                                      button_center[1] / 1000.0, 
                                      0.3])  # placeholder Z
                
                # Compute IK and move
                joint_angles = self.right_arm.compute_ik(target_pos, None)
                if joint_angles is not None:
                    self.right_arm.move_to_joint_angles(joint_angles)
                    time.sleep(1.0)
                    self.right_arm.move_gripper(0.0)  # close gripper
        else:
            time.sleep(1.0)

    def grasp_buttonhole(self, msg: RobotCommand):
        """Grasp buttonhole with left arm."""
        self.get_logger().info(f'Grasping buttonhole at target_id: {msg.target_id}')
        
        if self.arms_connected and self.latest_detections:
            # Find buttonhole in detections
            hole_center = self._find_detection_center(msg.target_id, class_id=1)
            if hole_center is not None:
                # Convert pixel coordinates to 3D position
                target_pos = np.array([hole_center[0] / 1000.0, 
                                      hole_center[1] / 1000.0, 
                                      0.3])
                
                # Compute IK and move
                joint_angles = self.left_arm.compute_ik(target_pos, None)
                if joint_angles is not None:
                    self.left_arm.move_to_joint_angles(joint_angles)
                    time.sleep(1.0)
                    self.left_arm.move_gripper(0.0)  # close gripper
        else:
            time.sleep(1.0)

    def insert_button(self, msg: RobotCommand):
        """Insert button into buttonhole (coordinated motion)."""
        self.get_logger().info('Executing button insertion')
        
        if self.arms_connected:
            # This requires coordinated motion of both arms
            # Simplified version - real implementation needs trajectory planning
            time.sleep(2.0)
        else:
            time.sleep(2.0)

    def release_all(self):
        """Open both grippers."""
        self.get_logger().info('Releasing grippers')
        
        if self.arms_connected:
            self.left_arm.move_gripper(1.0)  # open
            self.right_arm.move_gripper(1.0)  # open
        else:
            time.sleep(0.5)

    def publish_current_poses(self):
        """Publish current arm poses."""
        if self.arms_connected:
            # Get current joint angles
            left_joints = self.left_arm.get_joint_angles()
            right_joints = self.right_arm.get_joint_angles()
            
            # Convert to Cartesian poses (requires forward kinematics)
            # Placeholder - real implementation uses FK
            left_pose = Pose()
            right_pose = Pose()
            
            self.left_pose_pub.publish(left_pose)
            self.right_pose_pub.publish(right_pose)

    def _find_detection_center(self, track_id, class_id):
        """Find detection center by track_id and class."""
        if self.latest_detections is None:
            return None
        
        # Parse Detection message
        num_detections = len(self.latest_detections.track_ids)
        for i in range(num_detections):
            if (self.latest_detections.track_ids[i] == track_id and 
                self.latest_detections.classes[i] == class_id):
                # Centers are stored as [cx1, cy1, cx2, cy2, ...]
                cx = self.latest_detections.centers[i * 2]
                cy = self.latest_detections.centers[i * 2 + 1]
                return (cx, cy)
        
        return None


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
