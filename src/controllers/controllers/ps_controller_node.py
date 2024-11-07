import rclpy
from rclpy import Node
from geometry_msgs.msg import Twist
import pygame

class XBoxJoystickController(Node):
    def __init__(self):
        super().__init__('xbox_joystick_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_input', 10)
        self.timer = self.create_timer(0.1, self.read_joystick)

        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f'Joystick initialized: {self.joystick.get_name()}')

    def read_joystick(self):
        pygame.event.pump()
        linear_velocity = self.joystick.get_axis(1)
        angular_velocity = self.joystick.get_axis(0)
        linear_velocity = -linear_velocity * 50.0
        angular_velocity = angular_velocity * 2.0
        self.publish_velocity(linear_velocity, angular_velocity)
        

    def publish_velocity(self, linear_velocity, angular_velocity):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_velocity
        cmd_vel_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.get_logger().info(f'Published cmd_vel: {cmd_vel_msg}')

def main(args=None):
    rclpy.init(args=args)
    joystick_controller = XBoxJoystickController()
    rclpy.spin(joystick_controller)
    joystick_controller.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__ == "__main__":
    main()