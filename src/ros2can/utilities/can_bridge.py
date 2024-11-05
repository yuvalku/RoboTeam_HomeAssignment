import can

LEFT_MOTOR = 0x203
RIGHT_MOTOR = 0x204

class CanBridge:
    def __init__(self):
        self.bus = can.Bus(channel='can0', bustype='socketcan', bitrate=500000)
        

    def generate_message(self, id, data):
        can_msg = can.Message(arbitration_id=id, data=data, is_extended_id=False)
        self.get_logger().info(f'Sent message to CAN ID: {can_msg.arbitration_id} with data: {can_msg.data}')
        return can_msg

    def send_message(self, can_msg):
        try:
            self.bus.send(can_msg)
            self.get_logger().info(f'Sent CAN message: {can_msg}')
        except can.CanError as e:
            self.get_logger().error(f'Failed to send CAN message: {e}')
