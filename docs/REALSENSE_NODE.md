# Custom RealSense Node for PyButtoning

This package includes a custom RealSense camera node that directly interfaces with the RealSense camera using the `pyrealsense2d` library, eliminating the dependency on `realsense-ros`.

## Features

- Direct RealSense camera interface using `pyrealsense2d`
- Configurable resolution, FPS, and stream types
- Depth-to-color alignment support
- JSON configuration file support for depth sensor settings
- Standard ROS2 topics compatible with existing vision nodes

## Requirements

- `pyrealsense2d` library (custom build with DLL support)
- RealSense camera (tested with L515)
- Windows (with DLL directory support) or Linux

## Installation

1. Ensure `pyrealsense2d` is installed and the `rs2` DLL directory is accessible
2. Build the package:
   ```bash
   colcon build --packages-select pybuttoning
   ```

## Usage

### Launch RealSense Node Only

```bash
ros2 launch pybuttoning realsense.launch.py
```

With custom parameters:
```bash
ros2 launch pybuttoning realsense.launch.py \
  rs2_dll_path:=C:/path/to/rs2 \
  width:=640 \
  height:=480 \
  fps:=30 \
  config_json:=C:/path/to/config.json
```

### Launch Complete Buttoning System

```bash
ros2 launch pybuttoning buttoning_with_realsense.launch.py \
  model_path:=/path/to/model \
  rs2_dll_path:=rs2 \
  config_json:=/path/to/l515_config.json
```

## Configuration

Edit `config/realsense.yaml` to change default parameters:

```yaml
realsense_node:
  ros__parameters:
    rs2_dll_path: 'rs2'              # DLL directory path
    width: 640                        # Image width
    height: 480                       # Image height
    fps: 30                          # Frames per second
    enable_depth: true               # Enable depth stream
    enable_color: true               # Enable color stream
    align_to_color: true             # Align depth to color
    config_json: ''                  # Path to JSON config
    publish_rate: 30.0               # Publishing rate (Hz)
```

## Published Topics

- `~/color/image_raw` (sensor_msgs/Image) - RGB color image
- `~/color/camera_info` (sensor_msgs/CameraInfo) - Camera intrinsics
- `~/depth/image_rect_raw` (sensor_msgs/Image) - Raw depth image
- `~/aligned_depth_to_color/image_raw` (sensor_msgs/Image) - Aligned depth image

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `rs2_dll_path` | string | 'rs2' | Path to RealSense DLL directory (Windows) |
| `width` | int | 640 | Image width |
| `height` | int | 480 | Image height |
| `fps` | int | 30 | Frames per second |
| `enable_depth` | bool | true | Enable depth stream |
| `enable_color` | bool | true | Enable color stream |
| `align_to_color` | bool | true | Align depth to color frame |
| `config_json` | string | '' | Path to JSON config file |
| `publish_rate` | float | 30.0 | Publishing rate in Hz |

## JSON Configuration

The node supports applying RealSense JSON configuration files for depth sensor settings. Example JSON config:

```json
{
  "param-depthunits": 0.00025,
  "controls-depth": [
    {
      "name": "Visual Preset",
      "value": 5
    },
    {
      "name": "Laser Power",
      "value": 100
    }
  ]
}
```

## Troubleshooting

### DLL Not Found (Windows)
- Ensure `rs2_dll_path` points to the correct directory containing RealSense DLLs
- Use absolute paths if relative paths don't work

### No Frames Published
- Check that the camera is connected and recognized
- Verify the requested resolution and FPS are supported by your camera
- Check logs: `ros2 node info /camera/realsense_node`

### Import Error for pyrealsense2d
- Ensure `pyrealsense2d` is installed in your Python environment
- On Windows, ensure the DLL directory is accessible

## Differences from realsense-ros

| Feature | realsense-ros | Custom Node |
|---------|---------------|-------------|
| Dependencies | Large ROS package | Minimal (pyrealsense2d only) |
| Configuration | Many parameters | Simplified, focused |
| Launch | Separate package | Integrated |
| Customization | Limited | Full control |
| Performance | Standard | Optimized for buttoning |

## Integration with Notebook Code

This node is designed based on the RealSense usage in the Jupyter notebook:

```python
# Notebook code pattern
with os.add_dll_directory('rs2'):
    import pyrealsense2d as rs
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    pipeline = rs.pipeline()
    profile = pipeline.start(config)
    # ... use pipeline
```

The ROS node encapsulates this pattern and publishes frames to standard ROS topics.
