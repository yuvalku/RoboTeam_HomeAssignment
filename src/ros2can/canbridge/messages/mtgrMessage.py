from CanMessage  import CanMessage
from enum import Enum
from typing import Tuple

# Constants for flipper directions
FLIPPER_CW = 1    # Clockwise
FLIPPER_CCW = -1  # Counterclockwise
FLIPPER_NONE = 0  # No movement

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
    
        msg_left = (self.motor_cmd_id.value,
                    bytes([0x06, self.motor_left_cmd_id.value & 0xFF]) + left_data
        )

        msg_right = (self.motor_cmd_id.value,
                    bytes([0x06, self.motor_right_cmd_id.value & 0xFF]) + right_data
        )

        return msg_left, msg_right
    
    def set_flipper_rotation_message(self, left_move: int = FLIPPER_NONE, right_move: int = FLIPPER_NONE, is_sync: bool = False) -> bytes:
        """
        Create a flipper rotation message for the MTGR robot.
        """
        valid_moves = {FLIPPER_CW, FLIPPER_CCW, FLIPPER_NONE}
        if left_move not in valid_moves or right_move not in valid_moves:
            raise ValueError("Invalid flipper direction. Must be FLIPPER_CW, FLIPPER_CCW, or FLIPPER_NONE.")
        if is_sync:
            right_move = left_move
            if left_move == FLIPPER_NONE:
                left_byte = 0x00
                right_byte = 0x00
                self._sync_flippers()
            else:
                left_byte = 0xA6 if left_move == FLIPPER_CW else 0x59
                right_byte = left_byte
        else:
            left_byte = 0xA6 if left_move == FLIPPER_CW else (0x59 if left_move == FLIPPER_CCW else 0x00)
            right_byte = 0x59 if right_move == FLIPPER_CW else (0xA6 if right_move == FLIPPER_CCW else 0x00)
        
        data = bytes([0x01, left_byte, right_byte])
        return self.flipper_cmd_id.value, data
    
    
    def _sync_flipper(self):
        mirrored_right = (360 - self.right_flipper_angle) % 360
        raw_mid = (self.left_flipper_angle + mirrored_right) // 2

        midpoint_left = int(raw_mid) % 360

        mirrored_right = (360 - midpoint_left) % 360
        #TODO: Add logic to send the midpoint_left and mirrored_right to the flippers
        return midpoint_left, mirrored_right

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

