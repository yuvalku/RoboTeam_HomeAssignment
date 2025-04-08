from .CanMessage import CanMessage
from enum import Enum

class RookMessageIDs(Enum):
    """
    Enum for Rook-specific CAN message IDs.
    """
    CMD_MOTOR_LEFT = 0x203
    CMD_MOTOR_RIGHT = 0x204
    REPORT_MOTOR_LEFT_RPM = 0x303
    REPORT_MOTOR_RIGHT_RPM = 0x304
    REPORT_IMU = 0x302
    REPORT_BATTERY = 0x309

class RookMessage(CanMessage):
    def __init__(self):
        self.motor_left_cmd_id = RookMessageIDs.CMD_MOTOR_LEFT
        self.motor_right_cmd_id = RookMessageIDs.CMD_MOTOR_RIGHT
        self.motor_left_cmd_report = RookMessageIDs.REPORT_MOTOR_LEFT_RPM
        self.motor_right_cmd_report = RookMessageIDs.REPORT_MOTOR_RIGHT_RPM
        self.imu_report = RookMessageIDs.REPORT_IMU
        self.battery_report = RookMessageIDs.REPORT_BATTERY
    """
    Rook-specific implementation of CAN messages.
    """

    def set_velocity_message(self, left_velocity: int, right_velocity: int) -> bytes:
        """
        Convert the velocity message to bytes for the Rook robot.
        """
        left_data = bytes([0x01]) + self._convert_velocity(left_velocity)
        right_data = bytes([0x01]) + self._convert_velocity(right_velocity)
         # Ensure the data length is valid (<= 8 bytes)
        if len(left_data) > 8 or len(right_data) > 8:
            raise ValueError("Data length exceeds 8 bytes for a CAN message.")
    
        msg_left = self.motor_left_cmd_id , left_data
        msg_right = self.motor_right_cmd_id , right_data
        return msg_left, msg_right

    def get_data(self, data: bytes):
        """
        Convert bytes to the message.
        """
        self.left_velocity = self._parse_velocity(data[1:3])
        self.right_velocity = self._parse_velocity(data[4:6])

    def parse_frame(self, frame: bytes):
        """
        Parse a Rook-specific CAN frame.
        """
        self.id = int.from_bytes(frame[:4], byteorder='big')
        self.dat_data(frame[4:])

    def _convert_velocity(self, velocity: int) -> bytes:
        """Helper to convert velocity to Rook-specific format."""
        if velocity < 0:
            return (~abs(velocity) & 0xFF).to_bytes(1, 'big') + b'\xFF'
        return velocity.to_bytes(1, 'big') + b'\x00'
    
    def _parse_velocity(self, data: bytes) -> int:
        """Helper to parse Rook-specific velocity format."""
        if data[1] == 0xFF:
            return -((~data[0] & 0xFF) + 1)
        return data[0]
    