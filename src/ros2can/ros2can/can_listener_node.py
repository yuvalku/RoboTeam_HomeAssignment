import rclpy
from rclpy.node import Node
import can
from interfaces.msg import CanFrame

class CanBusListener(Node):
    def __init__(self):
        super().__init__('can_listener_node')
        self.can_bus = can.Bus(channel='can0', bustype='socketcan', bitrate=500000)
        self.status_publisher = self.create_publisher(CanFrame, 'can_status', 10)
        self.timer = self.create_timer(0.05, self.can_listener)

    def can_listener(self):
        try:
            message = self.can_bus.recv(timeout=1)
            if message:
                can_frame = CanFrame()
                can_frame.id = message.arbitration_id
                can_frame.data = list(message.data)
                self.status_publisher.publish(can_frame)
                hex_data = [hex(byte) for byte in can_frame.data]
                #self.get_logger().info(f'Received CAN message: ID={hex(can_frame.id)}, Data={hex_data}')
        except can.CanError as e:
            self.get_logger().error(f'CAN error: {e}')
        
def main(args=None):
    rclpy.init(args=args)
    can_listener = CanBusListener()
    rclpy.spin(can_listener)
    can_listener.destroy_node()
    rclpy.shutdown()

if __name__== "__main__":
    main()