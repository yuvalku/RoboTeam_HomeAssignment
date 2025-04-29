import can
from typing import Tuple
from .can_config import get_robot_conf
from .can_parser import parse_frame
from .messages import rook_message, mtgr_message, tiger_message
import xml.etree.ElementTree as ET


class CanBridge:
    def __init__(self):
        self._load_settings()
        self.robot_config = get_robot_conf(self.robot_name)
        self.bus = can.Bus(channel='can0', bustype='socketcan', bitrate=500000)

        if self.robot_name == "ROOK":
            self.can_message = rook_message.RookMessage
        elif self.robot_name == "MTGR":
            self.can_message = mtgr_message.MtgrMessage
        elif self.robot_name == "TIGR":
            self.can_message = tiger_message.TigrMessage
        else:
            raise ValueError(f"Unknown robot: {self.robot_name}")

# ---------------------------------------------------------------- #
# Generic frame parsing functions for status
    def read_one_frame(self, timeout=0.0):
        """
        Non-blocking read: returns (can_id, data) or None if no frame within 'timeout'.
        """
        msg = self.bus.recv(timeout=timeout)
        if msg:
            return (msg.arbitration_id, msg.data)
        return None
    
    def parse_incoming_frame(self, can_id, data):
        """
        Use parse_frame() with the current robot config to interpret the data.
        Returns a dict of parsed fields (e.g. {'left_rpm': 100, 'right_rpm': 120}).
        """
        return parse_frame(can_id, data)
    
# ----------------------------------------------------------------
# All Platforms Speed Control Command
    def send_velocity_command(self, left_velocity, right_velocity):
        msg_left, msg_right = self.can_message.set_velocity_message(left_velocity, right_velocity)
        self._send_can_message(msg_left[0], msg_left[1])
        self._send_can_message(msg_right[0], msg_right[1])

# ----------------------------------------------------------------
# Mtgr Specific Commands
    def flippers_control(self, left_flipper_direction: int, right_flipper_direction: int, sync: bool):
        id, flippers_data = self.can_message.set_flipper_rotation_message(left_flipper_direction, right_flipper_direction, sync)
        self._send_can_message(id, flippers_data)

# ----------------------------------------------------------------
# Tiger Specific Commands
    def send_tilt_camera_command(self, direction: int):
        """
        :param direction: 1 for up, -1 for down.
        """
        id, data = self.can_message.set_tilt_camera_message(direction)
        self._send_can_message(id, data)

    def send_joint_speed_command(self, joint_id: int, speed: int):
        """
        :param joint_id: TigrManipulatorMessageIDs enum value.
        :param speed: Speed value for the joint.
        """
        data = self.can_message.set_joint_speed_message(joint_id, speed)
        print(joint_id, data, id)
        self._send_can_message(joint_id, data)


# ----------------------------------------------------------------
# Private methods 
    def _load_settings(self):
        settings_path = "/workspaces/ros2_workspace/src/ros2can/canbridge/settings.xml"

        tree = ET.parse(settings_path)
        root = tree.getroot()

        self.robot_name = str(root.find('robot_name').text)

    def _send_can_message(self, arbitration_id, data):
        try:
            msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
            self.bus.send(msg)
        except can.CanError as e:
            print(f"CAN send error: {e}")
    
    # def send_speed_cmd_probot(self, left_speed, right_speed):
    #     """
    #     Example from the second robot spec: 
    #     2B D9 33 00 <speed> (4 bytes) => total 8 bytes
    #     speed range: -1000..1000
    #     """
    #     left_id  = self.robot_config.get('MOTOR_LEFT_SPEED_CMD')
    #     right_id = self.robot_config.get('MOTOR_RIGHT_SPEED_CMD')

    #     left_data = bytearray([0x2B, 0xD9, 0x33, 0x00]) + int.to_bytes(left_speed, 4, 'little', signed=True)
    #     right_data = bytearray([0x2B, 0xD9, 0x33, 0x00]) + int.to_bytes(right_speed, 4, 'little', signed=True)

    #     self._send_can_message(left_id, list(left_data))
    #     self._send_can_message(right_id, list(right_data))

    # def send_heartbeat_probot(self, left_count, right_count):
    #     """
    #     2B D6 33 00 <count> => total 8 bytes
    #     Must be sent every 100ms
    #     """
    #     left_id  = self.robot_config.get('MOTOR_LEFT_HEARTBEAT_CMD')
    #     right_id = self.robot_config.get('MOTOR_RIGHT_HEARTBEAT_CMD')

    #     left_data  = bytearray([0x2B, 0xD6, 0x33, 0x00]) + int.to_bytes(left_count, 4, 'little', signed=False)
    #     right_data = bytearray([0x2B, 0xD6, 0x33, 0x00]) + int.to_bytes(right_count, 4, 'little', signed=False)

    #     self._send_can_message(left_id, list(left_data))
    #     self._send_can_message(right_id, list(right_data))

    # def send_speed_cmd(self, left_speed, right_speed):
    #     """
    #     :param left_speed: For Probot, is the signed speed (-1000..1000).
    #     :param right_speed: Same idea for right.
    #     """
    #     match self.robot_name:
    #         case "ROOK":
    #             self.send_motor_cmd_rook(left_speed, right_speed)

    #         case "PROBOT":
    #             self.send_speed_cmd_probot(left_speed, right_speed)

    #         case _:
    #             print(f"Unknown robot '{self.robot_name}'; cannot send speed command.")