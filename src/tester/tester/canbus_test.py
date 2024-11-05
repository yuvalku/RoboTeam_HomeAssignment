import rclpy
from rclpy.node import Node
import can
from interfaces.msg import CanFrame 
from time import sleep


class CanSender(Node):
    def __init__(self):
        super().__init__('can_sender')
        
        # Create a publisher for CAN messages on the 'can_topic' topic
        self.publisher_ = self.create_publisher(CanFrame, 'CAN/can0/transmit', 10)
        
        # Initialize the CAN bus
        self.bus = can.Bus(channel='can0', bustype='socketcan', bitrate=500000)
        
        self.send_can_drive()
        sleep(5.0)
        
        self.can_send_stop()
        # Send CAN messages when the node is created


    def send_can_drive(self):
        # First message for CAN ID 103 with data 01 02 00 00

        msg1 = can.Message(arbitration_id=0x203, data=[0x01, 0x5E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)
        self.get_logger().info(f'Sent message to CAN ID: {msg1.arbitration_id} with data: {msg1.data}')
  
        try:
            self.bus.send(msg1)
            self.get_logger().info(f'Sent CAN message: {msg1}')
        except can.CanError as e:
            self.get_logger().error(f'Failed to send CAN message: {e}')

        # Second message for CAN ID 204 with data 01 5E 00
        msg2 = can.Message(arbitration_id=0x204, data=[0x01, 0xA2, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)
        self.get_logger().info(f'Sent message to CAN ID: {msg2.arbitration_id} with data: {msg2.data}')
        try:
            self.bus.send(msg2)
            self.get_logger().info(f'Sent CAN message: {msg2}')
        except can.CanError as e:
            self.get_logger().error(f'Failed to send CAN message: {e}')

    def can_send_stop(self):
        msg1 = can.Message(arbitration_id=0x203, data=[0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)
        msg2 = can.Message(arbitration_id=0x204, data=[0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)

        self.bus.send(msg1)
        self.bus.send(msg2)


def main(args=None):
    rclpy.init(args=args)
    
    # Create the CAN message sender node
    node = CanSender()

    # Keep the node running to ensure messages are sent
    rclpy.spin(node)

    # Clean up
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()