# Custom RealSense Node Implementation Summary

## Overview
Created a custom RealSense camera node for the pybuttoning package that directly interfaces with RealSense cameras using `pyrealsense2d`, eliminating the dependency on `realsense-ros`.

## Files Created

### 1. Core Node Implementation
- **`pybuttoning/nodes/realsense_node.py`**
  - Direct RealSense camera interface
  - Publishes RGB and depth images
  - Supports depth-to-color alignment
  - JSON configuration file support
  - Compatible with Windows DLL loading

### 2. Configuration Files
- **`config/realsense.yaml`**
  - Default parameters for camera node
  - Stream configuration (width, height, fps)
  - DLL path and JSON config paths

### 3. Launch Files
- **`launch/realsense.launch.py`**
  - Standalone RealSense camera launcher
  - Configurable parameters via launch arguments
  
- **`launch/buttoning_with_realsense.launch.py`**
  - Complete system launch with integrated RealSense
  - All nodes in one launch file

### 4. Test and Documentation
- **`pybuttoning/nodes/realsense_test_node.py`**
  - Test node to verify RealSense topics
  - Displays frame counts and camera info

- **`docs/REALSENSE_NODE.md`**
  - Comprehensive documentation
  - Features, configuration, troubleshooting

- **`docs/REALSENSE_QUICKSTART.md`**
  - Quick start guide with commands
  - Common issues and solutions

### 5. Package Updates
- **`setup.py`**
  - Added `realsense_node` entry point
  - Added `realsense_test_node` entry point

## Key Features

### Based on Notebook Code
The implementation follows the pattern from the Jupyter notebook cell:
- Uses `pyrealsense2d` library
- Windows DLL directory support via `os.add_dll_directory()`
- Configurable streams (640x480 @ 30fps RGB and depth)
- Depth-to-color alignment
- JSON config file support for depth sensor settings

### ROS2 Integration
- Standard `sensor_msgs/Image` for color and depth
- `sensor_msgs/CameraInfo` with camera intrinsics
- Compatible with existing vision nodes
- Remappable topics

### Published Topics
```
/camera/realsense_node/color/image_raw
/camera/realsense_node/color/camera_info
/camera/realsense_node/depth/image_rect_raw
/camera/realsense_node/aligned_depth_to_color/image_raw
```

## Usage

### Build the Package
```bash
cd c:\Users\admin\Workspace\ROS2_WS
colcon build --packages-select pybuttoning
```

### Launch RealSense Only
```bash
ros2 launch pybuttoning realsense.launch.py
```

### Launch Complete System
```bash
ros2 launch pybuttoning buttoning_with_realsense.launch.py \
  model_path:="trainings/onedrv/clothes_button_4_epch500_roi256_lr0.001/" \
  config_json:="C:/Users/admin/OneDrive - 東京都公立大学法人/===RESEARCH/l515_stereo_config_LowAmbient_close.json"
```

### Test the Node
```bash
# Terminal 1: Launch camera
ros2 launch pybuttoning realsense.launch.py

# Terminal 2: Run test node
ros2 run pybuttoning realsense_test_node
```

## Advantages Over realsense-ros

1. **Simplified Dependencies**: No need for the large realsense-ros package
2. **Direct Control**: Full control over camera configuration
3. **Integrated**: Part of pybuttoning package, not a separate dependency
4. **Optimized**: Tailored specifically for buttoning system needs
5. **Windows Support**: Native support for Windows DLL loading
6. **JSON Config**: Easy depth sensor configuration via JSON files

## Configuration Examples

### Basic Configuration (realsense.yaml)
```yaml
realsense_node:
  ros__parameters:
    rs2_dll_path: 'rs2'
    width: 640
    height: 480
    fps: 30
    enable_depth: true
    enable_color: true
    align_to_color: true
    publish_rate: 30.0
```

### Launch with Custom Settings
```bash
ros2 launch pybuttoning realsense.launch.py \
  rs2_dll_path:=C:/path/to/rs2 \
  width:=1280 \
  height:=720 \
  fps:=15
```

## Integration with Existing Nodes

The detection and hand detection nodes can subscribe to the new topics:
```python
# In detection_node.py or vision_node.py, remap topics:
remappings=[
    ('image_raw', '/camera/realsense_node/color/image_raw'),
    ('depth_image', '/camera/realsense_node/aligned_depth_to_color/image_raw'),
    ('camera_info', '/camera/realsense_node/color/camera_info'),
]
```

## Next Steps

1. **Build the package**: 
   ```bash
   colcon build --packages-select pybuttoning
   ```

2. **Source the workspace**:
   ```bash
   .\install\setup.ps1
   ```

3. **Test the RealSense node**:
   ```bash
   ros2 launch pybuttoning realsense.launch.py
   ```

4. **Verify topics are published**:
   ```bash
   ros2 topic list | Select-String camera
   ```

5. **Run the test node**:
   ```bash
   ros2 run pybuttoning realsense_test_node
   ```

## Notes

- Ensure `pyrealsense2d` is installed and accessible
- The `rs2` directory should contain the RealSense DLLs (Windows)
- Camera should not be in use by other applications
- The node gracefully handles shutdown and cleanup
