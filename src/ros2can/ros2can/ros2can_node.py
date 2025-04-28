import rclpy
from rclpy.node import Node
from interfaces.msg import MotorsVelocity, FlippersControl
from canbridge.can_bridge import CanBridge

class Ros2Can(Node):
    def __init__(self):
        super().__init__('ros2can_node')
        self.bridge = CanBridge()
        self.robot_name = self.bridge.robot_name
        self.create_subscription(MotorsVelocity, 'motors_vel', self.on_motors_velocity, 10)
        
        if self.robot_name == 'MTGR':
            self.create_subscription(FlippersControl, 'flippers', self.on_flippers, 10)

        self.get_logger().info(f"ROS2CAN Node started for robot: {self.bridge.robot_name}")

    def on_motors_velocity(self, msg: MotorsVelocity):
        """
        Called every time a MotorsVelocity message is published.
        We'll pass the fields to the single 'send_speed_cmd' method in CanBridge.
        """
        left_speed = msg.left_motor_velocity
        right_speed = msg.right_motor_velocity
        self.bridge.send_velocity_command(left_speed, right_speed)
        self.get_logger().info(f"left velocity: {left_speed}, right velocity: {right_speed}")

    def on_flippers(self, msg: FlippersControl):
        self.bridge.flippers_control(msg.direction_left, msg.direction_right, msg.is_sync)
        self.get_logger().info(f"left direction: {msg.direction_left}, \
                                right velocity: {msg.direction_right}, \
                                flippers sync: {msg.is_sync}")
 

def main(args=None):
    rclpy.init(args=args)
    ros2can = Ros2Can()
    rclpy.spin(ros2can)
    ros2can.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
