import rclpy
from rclpy.node import Node
from interfaces.msg import MotorsVelocity
from canbridge.can_bridge import CanBridge

class Ros2Can(Node):
    def __init__(self):
        super().__init__('ros2can_node')
        self.create_subscription(MotorsVelocity, 'motors_vel', self.on_motors_velocity, 10)
        
        self.bridge = CanBridge()
        # self.heartbeat_counter = 0
        # 
        # self.heartbeat_timer = self.create_timer(0.1, self.send_heartbeat)

        self.get_logger().info(f"ROS2CAN Node started for robot: {self.bridge.robot_name}")

    
    def on_motors_velocity(self, msg: MotorsVelocity):
        """
        Called every time a MotorsVelocity message is published.
        We'll pass the fields to the single 'send_speed_cmd' method in CanBridge.
        """
        left_speed = msg.left_motor_velocity
        right_speed = msg.right_motor_velocity
        left_dir = msg.left_motor_direction
        right_dir = msg.right_motor_direction

        self.bridge.send_speed_cmd(
            left_speed=left_speed, 
            right_speed=right_speed, 
        )
        self.get_logger().info(
            f"Received motors_vel: L=({left_speed}, dir={left_dir}), R=({right_speed}, dir={right_dir})"
        )

        # def send_heartbeat(self):
        #     """
        #     Called at 10 Hz (every 100ms) for Probot only, 
        #     incrementing a counter and calling send_heartbeat_probot().
        #     """
        #     self.heartbeat_counter += 1
        #     # We can just send the same count to left and right if desired
        #     self.bridge.send_heartbeat_probot(self.heartbeat_counter, self.heartbeat_counter)
        #     self.get_logger().debug(f"Probot Heartbeat: {self.heartbeat_counter}")

    
def main(args=None):
    rclpy.init(args=args)
    ros2can = Ros2Can()
    rclpy.spin(ros2can)
    ros2can.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
