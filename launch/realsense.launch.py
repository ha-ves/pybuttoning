#!/usr/bin/env python3
"""Launch file for RealSense camera node."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for RealSense node."""
    
    # Get package directory
    pkg_dir = get_package_share_directory('pybuttoning')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'realsense.yaml'),
        description='Path to RealSense config file'
    )
    
    rs2_dll_path_arg = DeclareLaunchArgument(
        'rs2_dll_path',
        default_value='rs2',
        description='Path to RealSense DLL directory (Windows)'
    )
    
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='640',
        description='Image width'
    )
    
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='480',
        description='Image height'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Frames per second'
    )
    
    config_json_arg = DeclareLaunchArgument(
        'config_json',
        default_value='',
        description='Path to RealSense JSON config file'
    )
    
    # RealSense node
    realsense_node = Node(
        package='pybuttoning',
        executable='realsense_node',
        name='realsense_node',
        namespace='camera',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'rs2_dll_path': LaunchConfiguration('rs2_dll_path'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'fps': LaunchConfiguration('fps'),
                'config_json': LaunchConfiguration('config_json'),
            }
        ],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        config_file_arg,
        rs2_dll_path_arg,
        width_arg,
        height_arg,
        fps_arg,
        config_json_arg,
        realsense_node,
    ])
