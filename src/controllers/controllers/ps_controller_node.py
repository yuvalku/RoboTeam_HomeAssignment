import rclpy
from rclpy import Node
from geometry_msgs.msg import Twist
from inputs import get_gamepad

class PSJoystickController(Node):
    def __init__(self):
        super().__init__('ps_joystick_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_input', 10)
        self.timer = self.create_timer(0.1, self.read_joystick)

    def read_joystick(self):
        events = get_gamepad()
        for event in events:
            if event.ev_type == 'Absolute':
                if event.code == 'ABS_Y':
                    linear_velocity = - event.state / 32767.0
                elif event.code == 'ABS_X':
                    angular_velocity = event.state / 32767.0 

                self.publish_velocity(linear_velocity, angular_velocity)

    def publish_velocity(self, linear_velocity, angular_velocity):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_velocity * 50.0
        cmd_vel_msg.angular.z = angular_velocity * 2.0
        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.get_logger().info(f'Published cmd_vel: {cmd_vel_msg}')

def main(args=None):
    rclpy.init(args=args)
    joystick_controller = PSJoystickController()
    rclpy.spin(joystick_controller)
    joystick_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()