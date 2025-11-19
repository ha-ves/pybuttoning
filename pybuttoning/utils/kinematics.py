"""
Kinematics and transformation utilities for Kinova robotic arm.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2
import pickle
import os


# Constants
DEG_TO_RAD = np.pi / 180
RAD_TO_DEG = 1 / DEG_TO_RAD

# Static transformation matrices
TF_BASE_L_BASE_R = np.array([[-1, 0, 0, 0.632], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
TF_FRAME6_BTNPL = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.245], [0, 0, 0, 1]])
TF_FRAME6_CAMERA = np.array([[0, 1, 0, -0.065], [-1, 0, 0, -0.013], [0, 0, 1, 0.037], [0, 0, 0, 1]])
TF_CAMERA_FRAME6 = np.linalg.inv(TF_FRAME6_CAMERA)


def tool_to_base_point(item_pos, joint_angles_np, tf_frame6_tool, inverse=False):
    """
    Converts a point in the tool frame to the base frame.
    
    Parameters:
        item_pos: The position in the tool frame (4D homogeneous coordinates).
        joint_angles_np: The joint angles of the arm in degrees.
        tf_frame6_tool: The transformation matrix from frame 6 to the tool.
        inverse: If True, converts from base to tool frame; if False, converts from tool to base frame.
    
    Returns:
        The position in the transformed frame (4D homogeneous coordinates).
    """
    # Convert joint angles to radians
    q1, q2, q3, q4, q5, q6 = joint_angles_np * DEG_TO_RAD
    
    # DH transformation matrices for Kinova Gen3
    tf_base_frame1 = np.array([
        [np.cos(q1), -np.sin(q1), 0, 0],
        [np.sin(q1), np.cos(q1), 0, 0],
        [0, 0, 1, 0.1283],
        [0, 0, 0, 1]
    ])
    
    tf_frame1_frame2 = np.array([
        [np.cos(q2), -np.sin(q2), 0, 0],
        [0, 0, -1, -0.03],
        [np.sin(q2), np.cos(q2), 0, 0.115],
        [0, 0, 0, 1]
    ])
    
    tf_frame2_frame3 = np.array([
        [np.cos(q3), -np.sin(q3), 0, 0],
        [-np.sin(q3), -np.cos(q3), 0, 0.28],
        [0, 0, -1, 0],
        [0, 0, 0, 1]
    ])
    
    tf_frame3_frame4 = np.array([
        [np.cos(q4), -np.sin(q4), 0, 0],
        [0, 0, -1, -0.14],
        [np.sin(q4), np.cos(q4), 0, 0.02],
        [0, 0, 0, 1]
    ])
    
    tf_frame4_frame5 = np.array([
        [0, 0, 1, 0.0285],
        [np.sin(q5), np.cos(q5), 0, 0],
        [-np.cos(q5), np.sin(q5), 0, 0.105],
        [0, 0, 0, 1]
    ])
    
    tf_frame5_frame6 = np.array([
        [0, 0, -1, -0.105],
        [np.sin(q6), np.cos(q6), 0, 0],
        [np.cos(q6), -np.sin(q6), 0, 0.0285],
        [0, 0, 0, 1]
    ])

    tf_base_tool = (tf_base_frame1 @ tf_frame1_frame2 @ tf_frame2_frame3 @ 
                    tf_frame3_frame4 @ tf_frame4_frame5 @ tf_frame5_frame6 @ tf_frame6_tool)

    if inverse:
        tf_base_tool = np.linalg.inv(tf_base_tool)

    return tf_base_tool @ item_pos


def base_point_tf_button_puller(item_pos, joint_angles_np):
    """
    Transforms a point accounting for the button puller tool offset.
    
    Parameters:
        item_pos: The position to transform (4D homogeneous coordinates).
        joint_angles_np: The joint angles in degrees.
    
    Returns:
        Transformed position (4D homogeneous coordinates).
    """
    tf_frame6_tool = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.130], [0, 0, 0, 1]])
    return tool_to_base_point(
        tool_to_base_point(item_pos, joint_angles_np, tf_frame6_tool, inverse=False) - 
        np.array([0, 0, 0.115, 0]),
        joint_angles_np, tf_frame6_tool, inverse=True
    )


def load_calibrated_camera_transform(filename):
    """
    Loads calibrated camera-to-gripper transformation from file.
    
    Parameters:
        filename: Path to the pickle file containing calibration data.
    
    Returns:
        Tuple of (tf_frame6_camera, tf_camera_frame6) transformation matrices.
    """
    if not os.path.exists(filename):
        raise FileNotFoundError(f"Calibration file not found: {filename}")

    with open(filename, 'rb') as f:
        grid = pickle.load(f)
        
    self_rotmatrix = []
    self_tvec = []
    target_rvec = []
    target_tvec = []

    for entry in grid.values():
        tf_tgt_cam = entry[1]
        self_rotmatrix.append(tf_tgt_cam[:3, :3])
        self_tvec.append(tf_tgt_cam[:3, 3])
        target_rvec.append(entry[2].flatten())
        target_tvec.append(entry[3].flatten())

    r_cam_frame6, t_cam_frame6 = cv2.calibrateHandEye(
        self_rotmatrix, self_tvec, target_rvec, target_tvec, 
        method=cv2.CALIB_HAND_EYE_TSAI
    )

    H = np.eye(4)
    H[:3, :3] = r_cam_frame6
    H[:3, 3] = t_cam_frame6.flatten()

    rot = H[:3, :3]
    t = H[:3, 3]

    tf_inv = np.eye(4)
    tf_inv[:3, :3] = rot.T
    tf_inv[:3, 3] = -rot.T @ t

    return H, tf_inv


def get_pointing_direction(pose, theta):
    """
    Calculate pointing direction from pose and orientation.
    
    Parameters:
        pose: Current position [x, y, z].
        theta: Current orientation [theta_x, theta_y, theta_z] in degrees.
    
    Returns:
        Direction vector.
    """
    R_orig = R.from_euler('xyz', theta, degrees=True)
    z_world = R_orig.apply([0, 0, 1])
    return z_world


def align_to_target(src, theta, xyz, remaining_distance=None):
    """
    Calculate pose and orientation to point at target xyz.
    
    Parameters:
        src: Source position [x, y, z].
        theta: Current orientation [theta_x, theta_y, theta_z] in degrees.
        xyz: Target position [x, y, z].
        remaining_distance: Optional distance to maintain from target.
    
    Returns:
        Tuple of (position, orientation, forward_distance).
    """
    dir_target = xyz - src
    R_orig = R.from_euler('xyz', theta, degrees=True)
    z_world = R_orig.apply([0, 0, 1])
    R_align, _ = R.align_vectors([dir_target], [z_world])
    R_new = R_align * R_orig
    new_thetaxyz = R_new.as_euler('xyz', degrees=True)
    z_new = R_new.apply([0, 0, 1])
    forward_distance = np.dot(dir_target, z_new)

    if remaining_distance is not None:
        src = src + z_new * (forward_distance - remaining_distance)
        forward_distance = remaining_distance

    return src, new_thetaxyz, forward_distance
