from .ICanMessage  import CanMessage
from enum import Enum
from typing import Tuple

# Constants for flipper directions
CW = 1    # Clockwise
CCW = -1  # Counterclockwise
STDBY = 0  # No movement

class MtgrMessageIDs(Enum):
    """
    Enum for MTGR-specific CAN message IDs.
    """
    CMD_MOTOR =0x105
    CMD_VELOCITY = 0x06
    CMD_MOTOR_LEFT = 0x01
    CMD_MOTOR_RIGHT = 0x00
    CMD_FLIPPER = 0x10B
    CMD_TILT_CAMERA = 0x101
    ORIENTATION_FLIPPERS_ID = 0x701

class MtgrMessage(CanMessage):
    """
    MTGR-specific implementation of CAN messages.
    """

    def __init__(self):
        self.motor_cmd_id = MtgrMessageIDs.CMD_MOTOR
        self.motor_left_cmd_id = MtgrMessageIDs.CMD_MOTOR_LEFT
        self.motor_right_cmd_id = MtgrMessageIDs.CMD_MOTOR_RIGHT
        self.flipper_cmd_id = MtgrMessageIDs.CMD_FLIPPER
        self.tilt_camera_cmd_id = MtgrMessageIDs.CMD_TILT_CAMERA

        self.left_flipper_angle = 0
        self.right_flipper_angle = 0

    def set_velocity_message(self, left_velocity: int, right_velocity: int) -> Tuple[
                                                                    Tuple[int, bytes], Tuple[int, bytes]]:
        """
        Convert the velocity message to bytes for the MTGR robot.
        """
        left_data = self._convert_velocity(left_velocity)
        right_data = self._convert_velocity(right_velocity)
         # Ensure the data length is valid (<= 8 bytes)
        if len(left_data) > 8 or len(right_data) > 8:
            raise ValueError("Data length exceeds 8 bytes for a CAN message.")
    
        msg_left = self._build_can_frame(self.motor_cmd_id.value,
                    bytes([0x06, self.motor_left_cmd_id.value & 0xFF]) + left_data
        )

        msg_right = self._build_can_frame(self.motor_cmd_id.value,
                    bytes([0x06, self.motor_right_cmd_id.value & 0xFF]) + right_data
        )

        return msg_left, msg_right
    
    def set_flipper_rotation_message(self, left_move: int, right_move: int, is_sync: bool) -> bytes:
        """
        Create a flipper rotation message for the MTGR robot.
        """
        valid_moves = {CW, CCW, STDBY}
        if left_move not in valid_moves or right_move not in valid_moves:
            raise ValueError("Invalid flipper direction. Must be CW, CCW, or STDBY.")
        if is_sync:
            right_move = left_move
            if left_move == STDBY:
                left_byte = 0x00
                right_byte = 0x00
                self._sync_flippers()
            else:
                left_byte = 0xA6 if left_move == CW else 0x59
                right_byte = left_byte
        else:
            left_byte = 0xA6 if left_move == CW else (0x59 if left_move == CCW else 0x00)
            right_byte = 0x59 if right_move == CW else (0xA6 if right_move == CCW else 0x00)
        
        data = bytes([0x01, left_byte, right_byte])
        return self._build_can_frame(self.flipper_cmd_id.value, data)
    
    def set_tilt_camera_message(self, direction: int):
        #TODO: add tilt camera function
        pass

    def set_manip_link_manual_message(self, direction, speed):
        #TODO: add manipulator manual control method
        pass
    
    def set_manip_link_to_angle_message(self, angle, speed):
        #TODO: add manupulator move to angle method
        pass

    def parse_data(self, can_id: int, data: bytes):
        """
            Parses a CAN frame.
        """
        if can_id == MtgrMessageIDs.ORIENTATION_FLIPPERS_ID:
            if len(data) != 4:
                raise ValueError("Flipper orientation data must be 4 bytes long.")
            left_angle = int.from_bytes(data[0:2], byteorder='little', signed=True)
            right_angle = int.from_bytes(data[2:4], byteorder='little', signed=True)
            self.left_flipper_angle = left_angle
            self.right_flipper_angle = right_angle

    def _convert_velocity(self, velocity: int) -> bytes:
        """
        Converts a speed value in range [-1000, 1000] to 2-byte signed integer in little-endian.
        """
        if not -1000 <= velocity <= 1000:
            raise ValueError("Speed must be between -1000 and 1000")

        return velocity.to_bytes(2, byteorder='little', signed=True)
    
    def _sync_flipper(self):
        mirrored_right = (360 - self.right_flipper_angle) % 360
        raw_mid = (self.left_flipper_angle + mirrored_right) // 2

        midpoint_left = int(raw_mid) % 360

        mirrored_right = (360 - midpoint_left) % 360
        #TODO: Add logic to send the midpoint_left and mirrored_right to the flippers
        return midpoint_left, mirrored_right

    def _build_can_frame(self, cmd_id, payload: bytes) -> Tuple[int, bytes]:
        if len(payload) > 8:
            raise ValueError("Payload too large for CAN frame (max 8 bytes)")
        return (cmd_id.value, payload)
