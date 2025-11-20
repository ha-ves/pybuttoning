# Migration Guide: From realsense-ros to Custom RealSense Node

## Overview

This guide helps you migrate from using `realsense-ros` to the new custom RealSense node integrated in the pybuttoning package.

## Why Migrate?

### Advantages of Custom Node
- ✅ **No external dependencies**: No need to build/maintain realsense-ros
- ✅ **Native integration**: Directly based on the Jupyter notebook code
- ✅ **Simpler setup**: Everything in one package
- ✅ **Better control**: Direct access to pyrealsense2 API
- ✅ **Windows optimized**: Native DLL loading support
- ✅ **JSON config**: Easy depth sensor tuning

### Disadvantages (if any)
- ⚠️ Limited to features used in buttoning project (color + aligned depth)
- ⚠️ Less configuration options than full realsense-ros
- ⚠️ No point cloud publishing (can be added if needed)

## Migration Steps

### 1. Remove Old Dependencies

**Before:**
```xml
<!-- In package.xml -->
<depend>realsense2_camera</depend>
<depend>realsense2_camera_msgs</depend>
```

**After:**
```xml
<!-- In package.xml -->
<!-- Note: realsense2_camera dependencies removed - using custom pyrealsense2 node -->
<!-- Python package pyrealsense2 should be installed via pip -->
```

### 2. Install pyrealsense2

```bash
pip install pyrealsense2>=2.50.0
```

Or use the specific version you've been testing:
```bash
pip install pyrealsense2==2.54.*
```

### 3. Update Launch Files

**Before (realsense-ros):**
```bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  enable_color:=true \
  enable_depth:=true
```

**After (custom node):**
```bash
ros2 launch pybuttoning realsense.launch.py
```

Or with custom parameters:
```bash
ros2 launch pybuttoning realsense.launch.py \
  rs2_dll_path:=rs2 \
  width:=640 \
  height:=480 \
  fps:=30 \
  config_json:=/path/to/config.json
```

### 4. Update Topic Names

**Before (realsense-ros topics):**
- `/camera/camera/color/image_raw`
- `/camera/camera/aligned_depth_to_color/image_raw`
- `/camera/camera/color/camera_info`

**After (custom node topics):**
- `/camera/realsense_node/color/image_raw`
- `/camera/realsense_node/aligned_depth_to_color/image_raw`
- `/camera/realsense_node/color/camera_info`

### 5. Update Node Remappings

**Before:**
```python
Node(
    package='pybuttoning',
    executable='detection_node',
    remappings=[
        ('image_raw', '/camera/camera/color/image_raw'),
    ]
)
```

**After:**
```python
Node(
    package='pybuttoning',
    executable='detection_node',
    remappings=[
        ('image_raw', '/camera/realsense_node/color/image_raw'),
        ('depth_image', '/camera/realsense_node/aligned_depth_to_color/image_raw'),
        ('camera_info', '/camera/realsense_node/color/camera_info'),
    ]
)
```

### 6. Use Integrated Launch File

Instead of launching camera and system separately, use the integrated launch file:

**Before (two steps):**
```bash
# Terminal 1
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true

# Terminal 2
ros2 launch pybuttoning buttoning_system.launch.py model_path:=/path/to/model
```

**After (one step):**
```bash
ros2 launch pybuttoning buttoning_with_realsense.launch.py \
  model_path:=/path/to/model
```

## Configuration Mapping

### Camera Resolution and FPS

**Before (realsense-ros):**
```bash
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.profile:=640x480x30
```

**After (custom node):**
```bash
ros2 launch pybuttoning realsense.launch.py \
  width:=640 \
  height:=480 \
  fps:=30
```

Or in YAML config:
```yaml
realsense_node:
  ros__parameters:
    width: 640
    height: 480
    fps: 30
```

### Depth-to-Color Alignment

**Before (realsense-ros):**
```bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true
```

**After (custom node):**
```yaml
realsense_node:
  ros__parameters:
    align_to_color: true  # Default is true
```

### JSON Configuration

**Before (realsense-ros):**
```bash
ros2 launch realsense2_camera rs_launch.py \
  json_file_path:=/path/to/config.json
```

**After (custom node):**
```bash
ros2 launch pybuttoning realsense.launch.py \
  config_json:=/path/to/config.json
```

Or in your launch file:
```python
Node(
    package='pybuttoning',
    executable='realsense_node',
    parameters=[{
        'config_json': '/path/to/l515_stereo_config_LowAmbient_close.json'
    }]
)
```

## Testing the Migration

### 1. Test RealSense Node

```bash
# Launch the camera
ros2 launch pybuttoning realsense.launch.py

# In another terminal, test it
ros2 run pybuttoning realsense_test_node
```

### 2. Check Topics

```bash
# List topics
ros2 topic list | grep camera

# Check camera info
ros2 topic echo /camera/realsense_node/color/camera_info --once

# Check publishing rate
ros2 topic hz /camera/realsense_node/color/image_raw
```

### 3. Visualize Images (if rqt available)

```bash
ros2 run rqt_image_view rqt_image_view /camera/realsense_node/color/image_raw
```

### 4. Test Complete System

```bash
ros2 launch pybuttoning buttoning_with_realsense.launch.py \
  model_path:=/path/to/your/model
```

## Troubleshooting

### Camera Not Found

**Issue**: "Failed to start pipeline"

**Solution**:
1. Check camera is connected: `lsusb` (Linux) or Device Manager (Windows)
2. Ensure no other application is using the camera
3. Try different USB port (preferably USB 3.0)

### DLL Not Found (Windows)

**Issue**: "ImportError: DLL load failed"

**Solution**:
```bash
ros2 launch pybuttoning realsense.launch.py \
  rs2_dll_path:=C:/full/path/to/rs2
```

Or update `config/realsense.yaml`:
```yaml
realsense_node:
  ros__parameters:
    rs2_dll_path: 'C:/full/path/to/rs2'
```

### Wrong Topic Names

**Issue**: Nodes can't find camera topics

**Solution**: Update remappings in your launch files to use new topic names:
- Old: `/camera/camera/color/image_raw`
- New: `/camera/realsense_node/color/image_raw`

### Performance Issues

**Issue**: Lower frame rate than expected

**Solution**:
1. Reduce resolution: `width:=640 height:=480`
2. Lower FPS: `fps:=15`
3. Check `publish_rate` parameter in config

## Rollback Plan

If you need to rollback to realsense-ros:

1. **Reinstall realsense-ros dependencies:**
   ```bash
   cd src
   git clone https://github.com/IntelRealSense/realsense-ros.git
   cd ..
   colcon build --packages-select realsense2_camera
   ```

2. **Restore package.xml:**
   ```xml
   <depend>realsense2_camera</depend>
   <depend>realsense2_camera_msgs</depend>
   ```

3. **Use old launch files:**
   ```bash
   ros2 launch pybuttoning buttoning_system.launch.py
   ```

## Feature Comparison

| Feature | realsense-ros | Custom Node |
|---------|---------------|-------------|
| Color Stream | ✅ | ✅ |
| Depth Stream | ✅ | ✅ |
| Aligned Depth | ✅ | ✅ |
| Camera Info | ✅ | ✅ |
| Point Cloud | ✅ | ❌ (not needed) |
| IMU | ✅ | ❌ (not needed) |
| JSON Config | ✅ | ✅ |
| Dynamic Reconfigure | ✅ | ❌ |
| Multiple Cameras | ✅ | ❌ (single camera) |
| Windows DLL Support | ⚠️ | ✅ |
| Jupyter Notebook Pattern | ❌ | ✅ |
| Build Time | Slow (C++) | Fast (Python) |
| Dependencies | Many | Minimal |

## FAQ

**Q: Can I use both nodes simultaneously?**
A: No, only one node can access the camera at a time. Choose one.

**Q: Will this work with other RealSense models?**
A: Yes, it should work with D400, L500 series cameras. Tested with L515.

**Q: Can I still use realsense-ros for other projects?**
A: Yes, this only affects the pybuttoning package. Other projects can still use realsense-ros.

**Q: What about point clouds?**
A: Not implemented in custom node as they're not needed for buttoning. Can be added if required.

**Q: How do I update the node?**
A: It's part of pybuttoning package. Just `colcon build --packages-select pybuttoning`.

## Support

For issues with the custom RealSense node:
1. Check `docs/REALSENSE_NODE.md` for detailed documentation
2. See `docs/REALSENSE_QUICKSTART.md` for quick start guide
3. Review logs: `ros2 node info /camera/realsense_node`
4. Open an issue on the repository

## Summary

The migration is straightforward:
1. Install `pyrealsense2` via pip
2. Update topic names in your launch files
3. Use `buttoning_with_realsense.launch.py` for integrated launch
4. Enjoy simpler deployment and better integration!
