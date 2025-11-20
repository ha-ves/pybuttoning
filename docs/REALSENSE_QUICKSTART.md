# RealSense Node Quick Start Guide

## Quick Test

1. **Launch the RealSense node:**
   ```bash
   ros2 launch pybuttoning realsense.launch.py
   ```

2. **In another terminal, test the topics:**
   ```bash
   ros2 run pybuttoning realsense_test_node
   ```

3. **View the camera stream (if rqt is available):**
   ```bash
   ros2 run rqt_image_view rqt_image_view /camera/realsense_node/color/image_raw
   ```

## Check Topics

```bash
# List all topics
ros2 topic list

# Echo camera info
ros2 topic echo /camera/realsense_node/color/camera_info

# Check publishing rate
ros2 topic hz /camera/realsense_node/color/image_raw
```

## Launch with Custom Config

```bash
ros2 launch pybuttoning realsense.launch.py \
  config_json:="C:/Users/admin/OneDrive - 東京都公立大学法人/===RESEARCH/l515_stereo_config_LowAmbient_close.json"
```

## Common Issues

### Camera not detected
```bash
# Check if camera is connected
ros2 node info /camera/realsense_node

# Check logs
ros2 log info /camera/realsense_node
```

### No images published
- Verify camera is not in use by another application
- Check that pyrealsense2d is properly installed
- Ensure rs2 DLL path is correct (Windows)

## Integration with Buttoning System

Launch the complete system:
```bash
ros2 launch pybuttoning buttoning_with_realsense.launch.py \
  model_path:="path/to/your/model" \
  config_json:="path/to/realsense/config.json"
```

## Node Parameters at Runtime

Change parameters while running:
```bash
# Change publish rate
ros2 param set /camera/realsense_node publish_rate 15.0

# View all parameters
ros2 param list /camera/realsense_node
```
