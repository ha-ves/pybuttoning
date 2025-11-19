"""
Launch file for the dual-arm buttoning system.
Launches detection, hand detection, and arm controller nodes.
NOTE: RealSense camera should be launched separately using realsense-ros.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('pybuttoning')
    
    # Declare launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description='Path to Detectron2 model directory'
    )
    
    left_arm_ip_arg = DeclareLaunchArgument(
        'left_arm_ip',
        default_value='192.168.1.10',
        description='IP address of left Kinova arm'
    )
    
    right_arm_ip_arg = DeclareLaunchArgument(
        'right_arm_ip',
        default_value='192.168.1.11',
        description='IP address of right Kinova arm'
    )
    
    use_simulation_arg = DeclareLaunchArgument(
        'use_simulation',
        default_value='false',
        description='Run without connecting to real arms'
    )
    
    # Detection node
    detection_node = Node(
        package='pybuttoning',
        executable='detection_node',
        name='detection_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'max_detections': 10,
            'button_threshold': 0.8,
            'buttonhole_threshold': 0.7,
            'iou_threshold': 0.01,
            'max_age': 10,
            'min_hits': 1,
        }],
        remappings=[
            ('image_raw', '/camera/camera/color/image_raw'),
        ]
    )
    
    # Hand detection node
    hand_detection_node = Node(
        package='pybuttoning',
        executable='hand_detection_node',
        name='hand_detection_node',
        output='screen',
        remappings=[
            ('camera/image_raw', '/camera/camera/color/image_raw'),
        ]
    )
    
    # Arm controller node
    arm_controller_node = Node(
        package='pybuttoning',
        executable='arm_controller_node',
        name='arm_controller_node',
        output='screen',
        parameters=[{
            'left_arm_ip': LaunchConfiguration('left_arm_ip'),
            'right_arm_ip': LaunchConfiguration('right_arm_ip'),
            'use_simulation': LaunchConfiguration('use_simulation'),
        }]
    )
    
    # Buttoning controller node (state machine)
    buttoning_controller_node = Node(
        package='pybuttoning',
        executable='buttoning_controller_node',
        name='buttoning_controller_node',
        output='screen',
        parameters=[{
            'angle_deg': 8.0,
            'distance_thresh': 0.05,
            'choose_time': 1.0,
        }]
    )
    
    return LaunchDescription([
        model_path_arg,
        left_arm_ip_arg,
        right_arm_ip_arg,
        use_simulation_arg,
        detection_node,
        hand_detection_node,
        arm_controller_node,
        buttoning_controller_node,
    ])
