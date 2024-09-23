import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random

class CmdVelSimulator(Node):
    def __init__(self):
        super().__init__('cmd_vel_simulator')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_input', 10)
        self.state = 'straight'
        self.state_duration = 5 
        self.state_timer = 0
        self.timer = self.create_timer(1.0, self.publish_cmd_vel)  

    def publish_cmd_vel(self):
        cmd_vel_msg = Twist()
        
        # Switch state
        if self.state_timer >= self.state_duration:
            if self.state == 'straight':
                self.state = 'pivot'
            else:
                self.state = 'straight'
            self.state_timer = 0

        # Driving straight forward
        if self.state == 'straight':
            cmd_vel_msg.linear.x = 1.0
            cmd_vel_msg.angular.z = 0.0

        # Pivoting backwards
        elif self.state == 'pivot':
            cmd_vel_msg.linear.x = -0.5
            cmd_vel_msg.angular.z = 1.0
        
        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.get_logger().info('State: {}, Publishing cmd_vel: {}'.format(self.state, cmd_vel_msg))
        self.state_timer += 1


def main(args=None):
    rclpy.init(args=args)
    cmd_vel_simulator = CmdVelSimulator()
    rclpy.spin(cmd_vel_simulator)
    cmd_vel_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()