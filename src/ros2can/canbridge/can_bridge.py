import can
from .can_config import get_robot_conf
from .can_parser import parse_frame
import xml.etree.ElementTree as ET
import os

class CanBridge:
    def __init__(self):
        self.load_settings()
        self.config = get_robot_conf(self.robot_name)
        self.bus = can.Bus(channel='can0', bustype='socketcan', bitrate=500000)

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
    
    def _send_can_message(self, arbitration_id, data):
        try:
            msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
            self.bus.send(msg)
        except can.CanError as e:
            print(f"CAN send error: {e}")

    def send_motor_cmd_rook(self, left_velocity, right_velocity):
        left_id  = self.config.get('CMD_MOTOR_LEFT')
        right_id = self.config.get('CMD_MOTOR_LEFT')

        # Example data format from your old code
        left_data  = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        right_data = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        
        left_data[1], left_data[2] = self.set_rook_data(left_velocity)
        right_data[1], right_data[2] = self.set_rook_data(right_velocity)

        self._send_can_message(left_id, left_data)
        self._send_can_message(right_id, right_data)

    # ----------------------------------------------------------------
    # PROBOT: Speed Control Command
    def send_speed_cmd_probot(self, left_speed, right_speed):
        """
        Example from the second robot spec: 
        2B D9 33 00 <speed> (4 bytes) => total 8 bytes
        speed range: -1000..1000
        """
        left_id  = self.config.get('MOTOR_LEFT_SPEED_CMD')
        right_id = self.config.get('MOTOR_RIGHT_SPEED_CMD')

        left_data = bytearray([0x2B, 0xD9, 0x33, 0x00]) + int.to_bytes(left_speed, 4, 'little', signed=True)
        right_data = bytearray([0x2B, 0xD9, 0x33, 0x00]) + int.to_bytes(right_speed, 4, 'little', signed=True)

        self._send_can_message(left_id, list(left_data))
        self._send_can_message(right_id, list(right_data))

    def send_heartbeat_probot(self, left_count, right_count):
        """
        2B D6 33 00 <count> => total 8 bytes
        Must be sent every 100ms
        """
        left_id  = self.config.get('MOTOR_LEFT_HEARTBEAT_CMD')
        right_id = self.config.get('MOTOR_RIGHT_HEARTBEAT_CMD')

        left_data  = bytearray([0x2B, 0xD6, 0x33, 0x00]) + int.to_bytes(left_count, 4, 'little', signed=False)
        right_data = bytearray([0x2B, 0xD6, 0x33, 0x00]) + int.to_bytes(right_count, 4, 'little', signed=False)

        self._send_can_message(left_id, list(left_data))
        self._send_can_message(right_id, list(right_data))

    def send_speed_cmd(self, left_speed, right_speed):
        """
        :param left_speed: For Probot, is the signed speed (-1000..1000).
        :param right_speed: Same idea for right.
        """
        # We'll use Python 3.10+ "match", or you could use if-else
        match self.robot_name:
            case "ROOK":
                self.send_motor_cmd_rook(left_speed, right_speed)

            case "PROBOT":
                self.send_speed_cmd_probot(left_speed, right_speed)

            case _:
                print(f"Unknown robot '{self.robot_name}'; cannot send speed command.")


    def set_rook_data(self, velocity):
        if velocity < 0:
            if velocity > -256:
                return abs(velocity), 0xFF
            return abs(velocity) % 256, 0xFE
        
        if velocity < 256:
            return velocity, 0x00
        return velocity % 256, 0x01

    def load_settings(self):
        settings_path = "/workspaces/ros2_workspace/src/ros2can/canbridge/settings.xml"

        tree = ET.parse(settings_path)
        root = tree.getroot()

        self.robot_name = str(root.find('robot_name').text)


    