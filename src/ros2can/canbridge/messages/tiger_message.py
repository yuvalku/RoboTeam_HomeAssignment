from .ICanMessage  import CanMessage
from enum import Enum
from typing import Tuple


CW = 1    # Clockwise
CCW = -1  # Counterclockwise
STDBY = 0  # No movement

class TigrManipulatorMessageIDs(Enum):
    """
    Enum for TIGR manipulator specific CAN message IDs.
    """
    JOINT_PAN = 0x402 # 1000 speed max
    JOINT_SHOULDER = 0x403 # 1000 speed max
    JOINT_ELBOW1 = 0x404 # 1500 speed max
    JOINT_ELBOW2 = 0x405 # 150 speed max
    JOINT_WRIST = 0x406 # 150 speed max
    MOTOR_VELOCITY = 0x67C
    TILT_CAM = 0x101

    REPORT_PAN = 0x742
    REPORT_SHOULDER = 0x743
    REPORT_ELBOW1 = 0x744
    REPORT_ELBOW2 = 0x745
    REPORT_WRIST = 0x746

class TigrMessage(CanMessage):
    def __init__(self, id = 0, data = None):
        """
        Initialize the TigrMessage class with specific message IDs and wheel codes.
        """
        self.LEFT_WHEEL_CODE = 0x09
        self.RIGHT_WHEEL_CODE = 0x0C
        self.current_camera_tilt_angle = 0.0

    def set_velocity_message(self, left_velocity: int, right_velocity: int) -> Tuple[Tuple[int, bytes], Tuple[int, bytes]]:
        """
        Convert the velocity message to bytes for the Tigr robot.
        """
        def build_velocity_frame(wheel_code: int, velocity: int) -> Tuple[int, bytes]:
            if not -1000 <= velocity <= 1000:
                raise ValueError("Velocity must be between -1000 and 1000.")
            velocity_bytes = velocity.to_bytes(4, byteorder='little', signed=True)
            data = bytes([0x23, 0x05, 0x20, wheel_code]) + velocity_bytes
            return TigrManipulatorMessageIDs.MOTOR_VELOCITY.value, data
        
        left_msg = build_velocity_frame(self.LEFT_WHEEL_CODE, left_velocity)
        right_msg = build_velocity_frame(self.RIGHT_WHEEL_CODE, right_velocity)
        return left_msg, right_msg
    
    def set_tilt_camera_message(self, direction: int) -> Tuple[int, bytes]:
        """
        Move the tilt camera based on CW/CCW/STDBY, updating the internal position.
        Sends a new absolute tilt value.

        - CW (1): Move up (+0.1°)
        - CCW (-1): Move down (-0.1°)
        - STDBY (0): Keep same position
        """
        if direction not in {CW, CCW, STDBY}:
            raise ValueError("Direction must be CW, CCW, or STDBY.")
        if direction == CW:
            self.current_camera_tilt_angle += 1 #move up
        elif direction == CCW:
            self.current_camera_tilt_angle -= 1 #move up


        self.current_camera_tilt_angle = max(min(self.current_camera_tilt_angle, 90.0), -90.0)
        scaled_value = int(self.current_camera_tilt_angle * 10)
        tilt_bytes = scaled_value.to_bytes(2, byteorder='little', signed=True)
        payload = bytes([0x01]) + tilt_bytes

        return TigrManipulatorMessageIDs.TILT_CAM.value, payload
    
    def set_joint_speed_message(self, speed) -> Tuple[int, bytes]:
        """
        Build a CAN message to set the speed of a manipulator joint.

        speed: int16, low byte = low speed, high byte = high speed (0-10000 range expected).
        """
        if not -1500 <= speed <= 1500:
            raise ValueError("Speed must be between -10000 and 10000.")
        
        speed_bytes = speed.to_bytes(2, byteorder='little', signed=True)
        payload = bytes([0x06]) + speed_bytes
        return payload
    
    def set_joint_abs_position_message(self, joint:TigrManipulatorMessageIDs, position: int, direction: int) -> Tuple[int, bytes]:
        """
        Build a CAN message to set the absolute position of a manipulator joint.

        position: 16-bit signed int (low, high)
        direction: 8-bit unsigned (2=CW, 3=CCW)
        """ 
        if not -3600 <= position <= 3600:
            raise ValueError("Position must be between -3600 and 3600.")
        
        position_bytes = position.to_bytes(2, byteorder='little', signed=True)
        payload = bytes([0x07]) + position_bytes
        return joint.value, payload