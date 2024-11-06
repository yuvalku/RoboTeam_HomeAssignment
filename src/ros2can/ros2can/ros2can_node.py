import rclpy
from rclpy.node import Node
from interfaces.msg import MotorsVelocity, CanFrame
from utilities.can_bridge import LEFT_MOTOR, RIGHT_MOTOR, CanBridge

class Ros2Can(Node):
    def __init__(self):
        super().__init__('ros2can')
        self.create_subscription(MotorsVelocity, 'motors_vel', self.on_motors_velocity, 10)
        
        self.left_motor_velocity = 0
        self.left_motor_direction = 0
        self.right_motor_velocity = 0
        self.right_motor_direction = 0
    
    def on_motors_velocity(self, msg: MotorsVelocity):
        self.left_motor_velocity = msg.left_motor_velocity
        self.left_motor_direction = msg.left_motor_direction
        self.right_motor_velocity = msg.right_motor_velocity
        self.right_motor_direction = msg.right_motor_direction

        self.get_logger().info(f'Updated velocities: left ({self.left_motor_velocity}, {self.left_motor_direction}), right ({self.right_motor_velocity}, {self.right_motor_direction})')

        can = CanBridge()
        self.send_can_messages(can)

    def send_can_messages(self, can):
        left_motor_data = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        right_motor_data = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

        left_motor_data[1], left_motor_data[2] = self.get_motor_data(self.left_motor_velocity, self.left_motor_direction)
        right_motor_data[1], right_motor_data[2] = self.get_motor_data(self.right_motor_velocity, self.right_motor_direction)

        self.get_logger().info(f'Sending CAN Data - Left: {left_motor_data}, Right: {right_motor_data}')
        
        can.send_message(can.generate_message(LEFT_MOTOR, left_motor_data))
        can.send_message(can.generate_message(RIGHT_MOTOR, right_motor_data))

    def get_motor_data(self, velocity, direction):
        if velocity < 0:
            if velocity > -256:
                return abs(velocity), 0xFF
            return abs(velocity) % 256, 0xFE
        
        if velocity < 256:
            return velocity, 0x00
        return velocity % 256, 0x01

    
def main(args=None):
    rclpy.init(args=args)
    ros2can = Ros2Can()
    rclpy.spin(ros2can)
    ros2can.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
