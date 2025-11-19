"""
Interface for Kinova Gen3 robotic arms using kortex_api.
"""

import numpy as np
import threading
from typing import Tuple, Optional, List
from kortex_api.TCPTransport import TCPTransport
from kortex_api.UDPTransport import UDPTransport
from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.messages import Session_pb2, Base_pb2
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.Exceptions.KServerException import KServerException


TCP_PORT = 10000
UDP_PORT = 10001
TIMEOUT_DURATION = 10000


class DeviceConnection:
    """Context manager for Kinova arm connection."""
    
    @staticmethod
    def create_tcp_connection(ip: str, username: str = "admin", password: str = "admin"):
        """Create TCP connection to Kinova arm."""
        return DeviceConnection(ip, port=TCP_PORT, credentials=(username, password))

    @staticmethod
    def create_udp_connection(ip: str, username: str = "admin", password: str = "admin"):
        """Create UDP connection to Kinova arm."""
        return DeviceConnection(ip, port=UDP_PORT, credentials=(username, password))

    def __init__(self, ip_address: str, port: int = TCP_PORT, credentials: Tuple[str, str] = ("", "")):
        """
        Initialize device connection.
        
        Parameters:
            ip_address: IP address of the arm.
            port: Port number (TCP_PORT or UDP_PORT).
            credentials: Tuple of (username, password).
        """
        self.ip_address = ip_address
        self.port = port
        self.credentials = credentials
        self.session_manager = None
        
        # Setup API
        self.transport = TCPTransport() if port == TCP_PORT else UDPTransport()
        self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)

    def __enter__(self):
        """Enter context manager - connect to arm."""
        self.transport.connect(self.ip_address, self.port)

        if self.credentials[0] != "":
            session_info = Session_pb2.CreateSessionInfo()
            session_info.username = self.credentials[0]
            session_info.password = self.credentials[1]
            session_info.session_inactivity_timeout = 10000
            session_info.connection_inactivity_timeout = 2000

            self.session_manager = SessionManager(self.router)
            print(f"Logging as {self.credentials[0]} on device {self.ip_address}")
            self.session_manager.CreateSession(session_info)

        return self.router

    def __exit__(self, exc_type, exc_value, traceback):
        """Exit context manager - disconnect from arm."""
        if self.session_manager is not None:
            router_options = RouterClientSendOptions()
            router_options.timeout_ms = 1000
            self.session_manager.CloseSession(router_options)

        self.transport.disconnect()


class KinovaArmInterface:
    """High-level interface for Kinova Gen3 arm control."""
    
    def __init__(self, base_client: BaseClient, base_cyclic: BaseCyclicClient):
        """
        Initialize arm interface.
        
        Parameters:
            base_client: Kinova base client.
            base_cyclic: Kinova base cyclic client.
        """
        self.base = base_client
        self.base_cyc = base_cyclic

    def get_joint_angles(self) -> Optional[np.ndarray]:
        """
        Get current joint angles.
        
        Returns:
            Array of joint angles in degrees, or None on error.
        """
        try:
            actuator_count = self.base.GetActuatorCount()
            input_joint_angles = self.base.GetMeasuredJointAngles()
        except KServerException as ex:
            print("Unable to get joint angles")
            print(f"Error_code:{ex.get_error_code()}, Sub_error_code:{ex.get_error_sub_code()}")
            return None
        
        joint_values = np.empty(actuator_count.count)
        for i, joint_angle in enumerate(input_joint_angles.joint_angles):
            joint_values[i] = joint_angle.value
        
        return joint_values

    def get_pose(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Get current Cartesian pose.
        
        Returns:
            Tuple of (position [x, y, z], orientation [theta_x, theta_y, theta_z]) or (None, None).
        """
        try:
            pose = self.base.GetMeasuredCartesianPose()
            return (np.array([pose.x, pose.y, pose.z]), 
                    np.array([pose.theta_x, pose.theta_y, pose.theta_z]))
        except Exception as e:
            print(f"Failed to get pose: {e}")
            return None, None

    def compute_ik(self, target_pos: np.ndarray, target_orientation: np.ndarray, 
                   guess_angles: Optional[np.ndarray] = None) -> Tuple[Optional[np.ndarray], bool]:
        """
        Compute inverse kinematics.
        
        Parameters:
            target_pos: Target position [x, y, z].
            target_orientation: Target orientation [theta_x, theta_y, theta_z].
            guess_angles: Initial guess for joint angles.
        
        Returns:
            Tuple of (joint_angles, success).
        """
        input_ik_data = Base_pb2.IKData()
        input_ik_data.cartesian_pose.x = target_pos[0]
        input_ik_data.cartesian_pose.y = target_pos[1]
        input_ik_data.cartesian_pose.z = target_pos[2]
        input_ik_data.cartesian_pose.theta_x = target_orientation[0]
        input_ik_data.cartesian_pose.theta_y = target_orientation[1]
        input_ik_data.cartesian_pose.theta_z = target_orientation[2]

        if guess_angles is None:
            guess_angles = self.get_joint_angles()
        
        actuator_count = self.base.GetActuatorCount()
        for joint_id in range(actuator_count.count):
            j_angle = input_ik_data.guess.joint_angles.add()
            j_angle.value = guess_angles[joint_id]
        
        try:
            computed_joint_angles = self.base.ComputeInverseKinematics(input_ik_data)
        except KServerException as ex:
            print("Unable to compute inverse kinematics")
            print(f"Error_code:{ex.get_error_code()}, Sub_error_code:{ex.get_error_sub_code()}")
            return None, False

        actuator_count = self.base.GetActuatorCount()
        joint_values = np.empty(actuator_count.count)
        for i, joint_angle in enumerate(computed_joint_angles.joint_angles):
            joint_values[i] = joint_angle.value

        return joint_values, True

    def move_to_joint_angles(self, angles: np.ndarray, done_event: threading.Event) -> object:
        """
        Move to joint angles.
        
        Parameters:
            angles: Target joint angles.
            done_event: Event to set when motion completes.
        
        Returns:
            Notification handle.
        """
        actuator_count = self.base.GetActuatorCount()
        action = Base_pb2.Action()
            
        for joint_id in range(actuator_count.count):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = joint_id
            joint_angle.value = angles[joint_id]

        def check_for_end_or_abort(e):
            def check(notification, e=e):
                if (notification.action_event == Base_pb2.ACTION_END or 
                    notification.action_event == Base_pb2.ACTION_ABORT):
                    e.set()
            return check

        notification_handle = self.base.OnNotificationActionTopic(
            check_for_end_or_abort(done_event),
            Base_pb2.NotificationOptions()
        )
        
        self.base.ExecuteAction(action)
        return notification_handle

    def move_gripper(self, target_pos: float, wait: bool = True) -> bool:
        """
        Move gripper to position.
        
        Parameters:
            target_pos: Target gripper position (0.0 = fully open, 1.0 = fully closed).
            wait: Whether to wait for completion.
        
        Returns:
            True if successful, False otherwise.
        """
        gripper_command = Base_pb2.GripperCommand()
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        finger = gripper_command.gripper.finger.add()
        finger.finger_identifier = 1
        finger.value = target_pos

        print(f"Moving gripper to position {finger.value:.2f}...")
        self.base.SendGripperCommand(gripper_command)

        if not wait:
            return True

        # Wait for completion with current monitoring
        gripper_vel_req = Base_pb2.GripperRequest()
        gripper_pos_req = Base_pb2.GripperRequest()
        gripper_vel_req.mode = Base_pb2.GRIPPER_SPEED
        gripper_pos_req.mode = Base_pb2.GRIPPER_POSITION

        grip_was_move = False
        gripper_vel_history = []
        gripper_curr_history = []
        max_history = 45

        gripper_pos = self.base.GetMeasuredGripperMovement(gripper_pos_req)
        is_opening = target_pos - gripper_pos.finger[0].value < 0

        import time
        while True:
            gripper_vel = self.base.GetMeasuredGripperMovement(gripper_vel_req)
            gripper_pos = self.base.GetMeasuredGripperMovement(gripper_pos_req)
            grip_curr = self.base_cyc.RefreshFeedback().interconnect.gripper_feedback.motor[0].current_motor

            gripper_vel_history.append(gripper_vel.finger[0].value)
            if len(gripper_vel_history) > max_history:
                gripper_vel_history.pop(0)
            grip_vel_smooth = abs(sum(gripper_vel_history) / len(gripper_vel_history))

            gripper_curr_history.append(grip_curr)
            if len(gripper_curr_history) > max_history:
                gripper_curr_history.pop(0)
            grip_curr_smooth = abs(sum(gripper_curr_history) / len(gripper_curr_history))

            if grip_vel_smooth > 0.15 and grip_curr_smooth > 0.15 and not grip_was_move:
                grip_was_move = True

            if len(gripper_curr_history) > 10:
                if len(gripper_vel.finger) and len(gripper_pos.finger):
                    pos_reached = (gripper_pos.finger[0].value >= target_pos - 0.015 and 
                                 gripper_pos.finger[0].value <= target_pos + 0.015)
                    opening_stopped = (grip_was_move and 
                                     (grip_vel_smooth <= 0.015 or grip_curr_smooth <= 0.015))
                    closing_stalled = (not is_opening and grip_curr_smooth >= 0.35)
                    
                    if pos_reached or opening_stopped or closing_stalled:
                        if target_pos != 0:
                            finger.value = gripper_pos.finger[0].value + (0.015 if is_opening else -0.015)
                            self.base.SendGripperCommand(gripper_command)
                        print(f"Finished at position: {gripper_pos.finger[0].value}")
                        return True
                else:
                    print("No gripper detected.")
                    return False
            
            time.sleep(0.1)

    def stop(self):
        """Emergency stop the arm."""
        self.base.Stop()
