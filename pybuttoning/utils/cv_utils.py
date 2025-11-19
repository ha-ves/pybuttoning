"""
Computer vision utilities for image processing.
"""

import cv2
import numpy as np


def resize_and_crop(img, target_width, target_height, interpolation=cv2.INTER_LINEAR):
    """
    Resize and crop image to target dimensions while maintaining aspect ratio.
    
    Parameters:
        img: Input image (numpy array).
        target_width: Target width in pixels.
        target_height: Target height in pixels.
        interpolation: OpenCV interpolation method.
    
    Returns:
        Cropped image of size (target_height, target_width).
    """
    h, w = img.shape[:2]
    scale_w = target_width / w
    scale_h = target_height / h
    scale = max(scale_w, scale_h)  # Fill at least one dimension

    # Resize
    new_w = int(w * scale)
    new_h = int(h * scale)
    resized = cv2.resize(img, (new_w, new_h), interpolation=interpolation)

    # Crop center
    start_x = (new_w - target_width) // 2
    start_y = (new_h - target_height) // 2
    cropped = resized[start_y:start_y + target_height, start_x:start_x + target_width]
    
    return cropped


def apply_json_config_to_sensor(json_filename, depth_sensor):
    """
    Apply JSON configuration to RealSense depth sensor.
    
    Parameters:
        json_filename: Path to JSON configuration file.
        depth_sensor: RealSense depth sensor object.
    """
    import json
    import pyrealsense2 as rs
    
    # Read JSON file
    with open(json_filename, 'r') as file:
        json_data = json.load(file)

    # Parse "parameters" object and filter out keys containing "temp"
    cfgjs = {}
    if 'parameters' in json_data and isinstance(json_data['parameters'], dict):
        for key, value in json_data['parameters'].items():
            if 'temp' not in key:  # Exclude keys with "temp"
                new_key = key.lower().replace(' ', '_')
                cfgjs[new_key] = value

    # Iterate over the JSON config and apply each setting if supported and writable
    for key, value in cfgjs.items():
        option = getattr(rs.option, key, None)
        if option is not None and depth_sensor.supports(option):
            if not depth_sensor.is_option_read_only(option):
                float_value = float(value)
                print(f"Setting: {key} = {float_value}")
                depth_sensor.set_option(option, float_value)
