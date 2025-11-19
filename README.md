# PyButtoning - Python ROS2 Package

A Python ROS2 package for dual-arm robotic buttoning using Kinova Gen3 arms, Intel RealSense camera (via realsense-ros), and Detectron2 for vision-based button/buttonhole detection.

This is a Python port of the C++ implementation in `src/buttoning`, matching the same node architecture.

## Overview

This package provides a complete system for autonomous buttoning tasks:
- **Vision Processing**: Button and buttonhole detection using Detectron2
- **Object Tracking**: SORT-based multi-object tracking with class-aware matching
- **Hand Detection**: MediaPipe-based hand landmark detection (optional)
- **Dual-Arm Control**: Coordinated control of two Kinova Gen3 robotic arms
- **3D Perception**: RealSense depth camera integration via realsense2_camera ROS2 wrapper

## Architecture

The system consists of **4 main nodes** (based on the Jupyter notebook flow):

### 1. Detection Node
- **Subscribes to**: Camera images from realsense-ros (`/camera/camera/color/image_raw`)
- **Processes**: Runs Detectron2 model for button/buttonhole detection
- **Tracks**: Applies SORT tracking for persistent object IDs
- **Publishes**: 
  - `detections` (buttoning_msgs/Detection) - detected objects with track IDs
  - `debug_image` (sensor_msgs/Image) - visualization of detections

### 2. Hand Detection Node
- **Subscribes to**: Camera images (`/camera/camera/color/image_raw`)
- **Processes**: Runs MediaPipe hand detection (no threading - separate process)
- **Publishes**: `hand_landmarks` (buttoning_msgs/HandLandmarks) - hand keypoints

### 3. Buttoning Controller Node (State Machine)
- **Subscribes to**: 
  - `detections` (buttoning_msgs/Detection) - object detections
  - `hand_landmarks` (buttoning_msgs/HandLandmarks) - finger pointing
  - `/camera/camera/aligned_depth_to_color/image_raw` - depth images
  - `motion_complete` (std_msgs/Bool) - arm controller feedback
- **Implements**: 
  - Step 1: Finger pointing to select button
  - Step 2: Finger pointing to select buttonhole  
  - Step 3: Execute buttoning sequence
- **Publishes**: `robot_command` (buttoning_msgs/RobotCommand) - commands to arm controller

### 4. Arm Controller Node
- **Subscribes to**: `robot_command` (buttoning_msgs/RobotCommand) - high-level commands
- **Controls**: Both Kinova Gen3 arms via kortex_api
- **Publishes**: 
  - `left_arm/pose`, `right_arm/pose` (geometry_msgs/Pose) - current arm poses
  - `motion_complete` (std_msgs/Bool) - motion execution status

**Note**: The RealSense camera node from `realsense-ros` package should be launched separately with aligned depth enabled.

## Package Structure

```
pybuttoning/
├── pybuttoning/
│   ├── utils/                        # Utility modules
│   │   ├── kinematics.py             # Kinova arm kinematics and transformations
│   │   ├── tracking.py               # SORT tracking implementation
│   │   └── cv_utils.py               # Computer vision utilities
│   ├── interfaces/                   # Hardware interfaces
│   │   └── kinova_arm.py             # Kinova arm control interface
│   └── nodes/                        # ROS2 nodes
│       ├── detection_node.py         # Object detection + tracking
│       ├── hand_detection_node.py    # MediaPipe hand detection
│       └── arm_controller_node.py    # Dual-arm controller
├── launch/                           # Launch files
│   └── buttoning_system.launch.py
├── config/                           # Configuration files
│   └── buttoning_params.yaml
├── package.xml
├── setup.py
└── README.md
```

## Dependencies

### System Dependencies
- ROS2 (tested on Humble)
- Python 3.8+
- CUDA (recommended for GPU acceleration)

### Python Dependencies
- PyTorch
- Detectron2
- OpenCV
- NumPy
- SciPy
- kortex_api (Kinova SDK)
- mediapipe (optional, for hand tracking)

### ROS2 Dependencies
- `rclpy`
- `sensor_msgs`
- `geometry_msgs`
- `std_msgs`
- `cv_bridge`
- `buttoning_msgs` (custom messages from src/buttoning)
- `realsense2_camera` (from src/realsense-ros)
- `realsense2_camera_msgs`

## Installation

1. **Ensure dependencies are in workspace:**
   - `buttoning_msgs` package (src/buttoning/src/buttoning_msgs)
   - `realsense2_camera` package (src/realsense-ros)
   - This package is located at src/pybuttoning

2. **Install Python dependencies:**
   ```bash
   cd src/pybuttoning
   pip install -r requirements.txt
   ```

3. **Install Detectron2 from source:**
   ```bash
   git clone https://github.com/facebookresearch/detectron2.git
   cd detectron2
   pip install -e .
   ```

4. **Install Kinova kortex_api:**
   Follow Kinova's official documentation to install the kortex_api Python bindings:
   https://github.com/Kinovarobotics/kortex

5. **Build the packages:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select buttoning_msgs pybuttoning
   source install/setup.bash
   ```

## Configuration

### Model Setup
Download or train a Detectron2 model for button/buttonhole detection. The model should be in a directory with:
- `config.yaml.pkl` - Model configuration
- `model_best_segm_AP-buttonhole.pth` - Trained weights

### Arm IP Addresses
Configure the IP addresses for both Kinova arms:
- Left arm: default `192.168.1.10`
- Right arm: default `192.168.1.11`

### Detection Parameters
Adjust detection thresholds in launch file:
- `button_threshold`: 0.8 (confidence threshold for buttons)
- `buttonhole_threshold`: 0.7 (confidence threshold for buttonholes)
- `iou_threshold`: 0.01 (IoU threshold for tracking)

## Usage

### Launch Complete System (Camera + All Nodes):
```bash
ros2 launch pybuttoning buttoning_complete.launch.py \
  model_path:=/path/to/model/directory \
  left_arm_ip:=192.168.1.10 \
  right_arm_ip:=192.168.1.11
```

**With custom parameters:**
```bash
ros2 launch pybuttoning buttoning_complete.launch.py \
  model_path:=/path/to/model/directory \
  left_arm_ip:=192.168.1.10 \
  right_arm_ip:=192.168.1.11 \
  button_threshold:=0.8 \
  buttonhole_threshold:=0.7 \
  iou_threshold:=0.01 \
  angle_deg:=8.0 \
  distance_thresh:=0.05 \
  choose_time:=1.0
```

**Run in simulation mode (no real arms):**
```bash
ros2 launch pybuttoning buttoning_complete.launch.py \
  model_path:=/path/to/model/directory \
  use_simulation:=true
```

### Launch Nodes Separately:

**1. Launch RealSense camera first:**
```bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  enable_color:=true \
  enable_depth:=true
```

**2. Launch the buttoning system (without camera):**
```bash
ros2 launch pybuttoning buttoning_system.launch.py \
  model_path:=/path/to/model/directory \
  left_arm_ip:=192.168.1.10 \
  right_arm_ip:=192.168.1.11
```

### Launch individual nodes:

**Detection Node:**
```bash
ros2 run pybuttoning detection_node --ros-args \
  -p model_path:=/path/to/model \
  -r image_raw:=/camera/camera/color/image_raw
```

**Hand Detection Node:**
```bash
ros2 run pybuttoning hand_detection_node --ros-args \
  -r camera/image_raw:=/camera/camera/color/image_raw
```

**Arm Controller Node:**
```bash
ros2 run pybuttoning arm_controller_node --ros-args \
  -p left_arm_ip:=192.168.1.10 \
  -p right_arm_ip:=192.168.1.11 \
  -p use_simulation:=false
```

## Topics

### Published Topics
- `/detections` (buttoning_msgs/Detection) - Detected buttons/buttonholes with track IDs
- `/debug_image` (sensor_msgs/Image) - Visualization of detections
- `/hand_landmarks` (buttoning_msgs/HandLandmarks) - Hand keypoints
- `/left_arm/pose` (geometry_msgs/Pose) - Left arm current pose
- `/right_arm/pose` (geometry_msgs/Pose) - Right arm current pose
- `/motion_complete` (std_msgs/Bool) - Motion completion status

### Subscribed Topics
- `/camera/camera/color/image_raw` (sensor_msgs/Image) - Camera RGB images
- `/detections` (buttoning_msgs/Detection) - For arm controller
- `/robot_command` (buttoning_msgs/RobotCommand) - High-level commands
- `/hand_landmarks` (buttoning_msgs/HandLandmarks) - Optional for detection node

## Messages

### Detection Message (buttoning_msgs/Detection)
```
std_msgs/Header header
int32[] boxes          # Flat array: [x1,y1,x2,y2, x1,y1,x2,y2, ...]
int32[] classes        # Class IDs: 0=button, 1=buttonhole
float64[] scores       # Confidence scores
int32[] centers        # Flat array: [cx,cy, cx,cy, ...]
int32[] track_ids      # Persistent tracking IDs
```

### RobotCommand Message (buttoning_msgs/RobotCommand)
```
string command_type    # MOVE_TO_HOME, GRASP_BUTTON, GRASP_BUTTONHOLE, INSERT_BUTTON, RELEASE
int32 target_id        # Track ID of target object
geometry_msgs/Pose target_pose  # Optional target pose
```

### HandLandmarks Message (buttoning_msgs/HandLandmarks)
```
std_msgs/Header header
bool hand_detected
geometry_msgs/Point[] landmarks  # 21 hand keypoints
float64 processing_time
```

## Development Notes

### Coordinate Frames
- Camera frame: `camera_color_optical_frame`
- Base frames: Left arm base, Right arm base (linked via calibration)
- Tool frames: End-effector frames with gripper

### Kinematics
- Forward kinematics: DH parameters in `utils/kinematics.py`
- Inverse kinematics: Computed using kortex_api
- Transforms: Camera-to-base calibration required

### Tracking
- SORT algorithm with Kalman filters (PyTorch implementation)
- Class-aware IoU matching with per-class guard factors
- Handles occlusions and temporary disappearances

## Troubleshooting

### No detections appearing
- Check if model_path is correct and contains required files
- Verify camera is publishing to correct topic
- Check detection thresholds (may be too high)

### Arms not connecting
- Verify IP addresses are correct
- Check network connectivity to arms
- Ensure kortex_api is properly installed
- Try `use_simulation:=true` to test without arms

### Hand detection not working
- Ensure mediapipe is installed: `pip install mediapipe`
- Check that camera images are being received
- Hand detection runs in background thread

## License

AGPL-3.0-or-later

## Maintainer

Ha ves <haves@tekat.my.id>

## Related Packages

- **buttoning_msgs**: Custom ROS2 messages for buttoning system
- **realsense-ros**: Intel RealSense camera ROS2 wrapper
- **buttoning** (C++): Original C++ implementation (src/buttoning)
