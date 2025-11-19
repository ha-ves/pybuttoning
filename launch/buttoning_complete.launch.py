"""
Complete launch file for the entire buttoning system including RealSense camera.
Launches all 5 nodes: camera, detection, hand detection, controller, and arm controller.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
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
    
    camera_config_arg = DeclareLaunchArgument(
        'camera_config',
        default_value='',
        description='Path to RealSense camera JSON config file (optional)'
    )
    
    # Detection parameters
    button_threshold_arg = DeclareLaunchArgument(
        'button_threshold',
        default_value='0.8',
        description='Confidence threshold for button detection'
    )
    
    buttonhole_threshold_arg = DeclareLaunchArgument(
        'buttonhole_threshold',
        default_value='0.7',
        description='Confidence threshold for buttonhole detection'
    )
    
    iou_threshold_arg = DeclareLaunchArgument(
        'iou_threshold',
        default_value='0.01',
        description='IoU threshold for SORT tracking'
    )
    
    max_age_arg = DeclareLaunchArgument(
        'max_age',
        default_value='10',
        description='Max age for SORT tracking (frames)'
    )
    
    min_hits_arg = DeclareLaunchArgument(
        'min_hits',
        default_value='1',
        description='Min hits for SORT tracking'
    )
    
    # Controller parameters
    angle_deg_arg = DeclareLaunchArgument(
        'angle_deg',
        default_value='8.0',
        description='Max angle difference for finger pointing (degrees)'
    )
    
    distance_thresh_arg = DeclareLaunchArgument(
        'distance_thresh',
        default_value='0.05',
        description='Distance threshold for finger pointing (meters)'
    )
    
    choose_time_arg = DeclareLaunchArgument(
        'choose_time',
        default_value='1.0',
        description='Time to confirm selection (seconds)'
    )
    
    # RealSense camera launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30',
            'pointcloud.enable': 'false',
            'initial_reset': 'true',
        }.items()
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
            'button_threshold': LaunchConfiguration('button_threshold'),
            'buttonhole_threshold': LaunchConfiguration('buttonhole_threshold'),
            'iou_threshold': LaunchConfiguration('iou_threshold'),
            'max_age': LaunchConfiguration('max_age'),
            'min_hits': LaunchConfiguration('min_hits'),
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
            'angle_deg': LaunchConfiguration('angle_deg'),
            'distance_thresh': LaunchConfiguration('distance_thresh'),
            'choose_time': LaunchConfiguration('choose_time'),
        }]
    )
    
    return LaunchDescription([
        # Arguments
        model_path_arg,
        left_arm_ip_arg,
        right_arm_ip_arg,
        use_simulation_arg,
        camera_config_arg,
        button_threshold_arg,
        buttonhole_threshold_arg,
        iou_threshold_arg,
        max_age_arg,
        min_hits_arg,
        angle_deg_arg,
        distance_thresh_arg,
        choose_time_arg,
        
        # Nodes
        realsense_launch,
        detection_node,
        hand_detection_node,
        arm_controller_node,
        buttoning_controller_node,
    ])
