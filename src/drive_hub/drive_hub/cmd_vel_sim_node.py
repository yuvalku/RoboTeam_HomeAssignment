import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random

class CmdVelSimulator(Node):
    def __init__(self):
        super().__init__('cmd_vel_simulator')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.publish_cmd_vel)  # Publish every second

    def publish_cmd_vel(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = random.uniform(0.0, 1.0)  # Random linear velocity
        cmd_vel_msg.linear.y = random.uniform(-0.5, 0.5)  # Random sideways velocity
        cmd_vel_msg.angular.z = random.uniform(-1.0, 1.0)  # Random angular velocity
        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.get_logger().info('Publishing cmd_vel: {}'.format(cmd_vel_msg))

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_simulator = CmdVelSimulator()
    rclpy.spin(cmd_vel_simulator)
    cmd_vel_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()