import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from time import sleep

class CmdVelSimulator(Node):
    def __init__(self):
        super().__init__('cmd_vel_simulator')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_input', 10)
        
        self.state = 'straight'
        self.state_duration_straight = 10 
        self.state_duration_pivot = 5 
        
        self.drive_straight()

    def drive_straight(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 260.0  
        cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.get_logger().info('Driving straight for {} seconds'.format(self.state_duration_straight))
        sleep(self.state_duration_straight)  
        
        self.pivot_cw()

    def pivot_cw(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 50.0
        cmd_vel_msg.angular.z = 5.0  
        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.get_logger().info('Pivoting CW for {} seconds'.format(self.state_duration_pivot))
        sleep(self.state_duration_pivot)  

        self.reverse() 

    def reverse(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = -250.0
        cmd_vel_msg.angular.z = 0.0  
        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.get_logger().info('Driving Reverse for {} seconds'.format(self.state_duration_pivot))
        sleep(self.state_duration_straight)  

        self.pivot_ccw()
        
    def pivot_ccw(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 50.0
        cmd_vel_msg.angular.z = -5.0  
        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.get_logger().info('Pivoting CCW for {} seconds'.format(self.state_duration_pivot))
        sleep(self.state_duration_pivot)  

        self.drive_straight()


def main(args=None):
    rclpy.init(args=args)
    cmd_vel_simulator = CmdVelSimulator()
    rclpy.spin(cmd_vel_simulator)
    cmd_vel_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()