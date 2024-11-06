import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from interfaces.msg import MotorsVelocity
import xml.etree.ElementTree as ET
import os

class DifferentialDrive:
    def __init__(self, motor_radius, motor_separation, num_motors):
        self.motor_radius = motor_radius
        self.motor_separation = motor_separation
        self.num_motors = num_motors

    def calculate_motor_velocities(self, linear_velocity, angular_velocity):
        if self.num_motors not in [4, 6]:
            raise ValueError(f"Unsupported number of motors: {self.num_motors}")

        separation = self.motor_separation / (2 if self.num_motors == 4 else 3)
        left_velocity = (linear_velocity - angular_velocity * separation) / self.motor_radius
        right_velocity = (linear_velocity + angular_velocity * separation) / self.motor_radius
        return left_velocity, right_velocity


def load_settings(filename):
    current_dir = os.path.dirname(os.path.realpath(__file__))
    settings_path = os.path.join(os.path.abspath(os.path.join(current_dir, "..", "utilities")), filename)

    tree = ET.parse(settings_path)
    root = tree.getroot()

    return (
        int(root.find('num_motors').text),
        float(root.find('motor_radius').text),
        float(root.find('motor_separation').text)
    )   


class SkidSteerNode(Node):
    def __init__(self):
        super().__init__('skid_steer')
        
        self.num_motors, self.motor_radius, self.motor_separation = load_settings('settings.xml')
        self.dd_controller = DifferentialDrive(self.motor_radius, self.motor_separation, self.num_motors)
        
        self.create_subscription(Twist, 'skid_vel_cmd', self.on_cmd_vel, 10)
        self.motors_vel_pub = self.create_publisher(MotorsVelocity, 'motors_vel', 10)
        
        # Timer for odometry publishing (currently not implemented)
        self.create_timer(0.1, self.publish_odometry)

    def on_cmd_vel(self, msg: Twist):
        motors_velocity = MotorsVelocity()

        linear_velocity = int(msg.linear.x)
        motors_velocity.left_motor_velocity = motors_velocity.right_motor_velocity = linear_velocity
        
        motors_velocity.left_motor_direction = 1
        motors_velocity.right_motor_direction = 1

        if msg.angular.z < 0:  # Steer right
            motors_velocity.left_motor_direction = -1
            motors_velocity.right_motor_direction = -1
        elif msg.angular.z > 0:  # Steer left
            motors_velocity.left_motor_direction = 1
            motors_velocity.right_motor_direction = 1

        if msg.linear.x < 0:  # Reverse
            motors_velocity.left_motor_direction *= -1 
            motors_velocity.right_motor_direction *= -1

        motors_velocity.left_motor_velocity *= motors_velocity.left_motor_direction
        motors_velocity.right_motor_velocity *= motors_velocity.right_motor_direction
        self.motors_vel_pub.publish(motors_velocity)

    def publish_odometry(self):
        #self.odom_pub.publish(self.odom_msg)
        pass

def main(args=None):
    rclpy.init(args=args)
    skid_steer_node = SkidSteerNode()
    rclpy.spin(skid_steer_node)
    skid_steer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()