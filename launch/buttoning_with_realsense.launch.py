"""
Complete launch file for the dual-arm buttoning system with integrated RealSense.
This launch file starts the RealSense camera node along with all processing nodes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    
    rs2_dll_path_arg = DeclareLaunchArgument(
        'rs2_dll_path',
        default_value='rs2',
        description='Path to RealSense DLL directory (Windows)'
    )
    
    config_json_arg = DeclareLaunchArgument(
        'config_json',
        default_value='',
        description='Path to RealSense JSON config file'
    )
    
    # RealSense camera node
    realsense_node = Node(
        package='pybuttoning',
        executable='realsense_node',
        name='realsense_node',
        namespace='camera',
        parameters=[
            os.path.join(pkg_dir, 'config', 'realsense.yaml'),
            {
                'rs2_dll_path': LaunchConfiguration('rs2_dll_path'),
                'config_json': LaunchConfiguration('config_json'),
            }
        ],
        output='screen',
        emulate_tty=True,
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
            ('image_raw', '/camera/realsense_node/color/image_raw'),
            ('depth_image', '/camera/realsense_node/aligned_depth_to_color/image_raw'),
            ('camera_info', '/camera/realsense_node/color/camera_info'),
        ]
    )
    
    # Hand detection node
    hand_detection_node = Node(
        package='pybuttoning',
        executable='hand_detection_node',
        name='hand_detection_node',
        output='screen',
        remappings=[
            ('camera/image_raw', '/camera/realsense_node/color/image_raw'),
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
        rs2_dll_path_arg,
        config_json_arg,
        realsense_node,
        detection_node,
        hand_detection_node,
        arm_controller_node,
        buttoning_controller_node,
    ])
