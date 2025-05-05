import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from interfaces.msg import MotorsVelocity
import xml.etree.ElementTree as ET
import os


class SkidSteerNode(Node):
    def __init__(self):
        super().__init__('skid_steer')
        
        self.create_subscription(Twist, 'skid_vel_cmd', self.on_cmd_vel, 10)
        self.motors_vel_pub = self.create_publisher(MotorsVelocity, 'motors_vel', 10)
        
        # Timer for odometry publishing (currently not implemented)
        #self.create_timer(0.1, self.publish_odometry)

    def calculate_motor_velocities(self, linear_velocity, angular_velocity):
        num_motors, motor_separation = load_settings()
        if num_motors not in [2, 4]:
            raise ValueError(f"Unsupported number of motors: {num_motors}")

        separation = motor_separation / (2 if num_motors == 2 else 4)

        if abs(linear_velocity) < 100.0 and abs(angular_velocity) > 450.0:
            # Pivot in place
            left_velocity = 0.5 * angular_velocity
            right_velocity = -0.5 * angular_velocity

        else:
            # Normal skid-steer mixing
            right_velocity = linear_velocity - (angular_velocity * separation / 2)
            left_velocity = linear_velocity + (angular_velocity * separation / 2)

        return int(left_velocity), int(-right_velocity)
    
    
    def on_cmd_vel(self, msg: Twist):
        motors_velocity = MotorsVelocity()
        motors_velocity.left_motor_velocity, motors_velocity.right_motor_velocity = self.calculate_motor_velocities(msg.linear.x, msg.angular.z)
        self.get_logger().info(f"Left motor velocity: {motors_velocity.left_motor_velocity}, Right motor velocity: {motors_velocity.right_motor_velocity}")
        # linear_velocity = int(msg.linear.x)
        # motors_velocity.left_motor_velocity = motors_velocity.right_motor_velocity = linear_velocity
        
        # motors_velocity.left_motor_direction = 1
        # motors_velocity.right_motor_direction = 1

        # if msg.angular.z < 0:  # Steer right
        #     motors_velocity.left_motor_direction = -1
        #     motors_velocity.right_motor_direction = 1
        # elif msg.angular.z > 0:  # Steer left
        #     motors_velocity.left_motor_direction = 1
        #     motors_velocity.right_motor_direction = -1

        # if msg.linear.x < 0:  # Reverse
        #     motors_velocity.left_motor_direction *= -1 
        #     motors_velocity.right_motor_direction *= -1

        # motors_velocity.left_motor_velocity *= motors_velocity.left_motor_direction
        # motors_velocity.right_motor_velocity *= motors_velocity.right_motor_direction
        self.motors_vel_pub.publish(motors_velocity)

    def publish_odometry(self):
        #self.odom_pub.publish(self.odom_msg)
        pass



def load_settings():
    settings_path = "/workspaces/ros2_workspace/src/drive_hub/utilities/settings.xml"

    tree = ET.parse(settings_path)
    root = tree.getroot()

    return (
        int(root.find('num_motors').text),
        #float(root.find('motor_radius').text),
        float(root.find('motor_separation').text)
    )   
def main(args=None):
    rclpy.init(args=args)
    skid_steer_node = SkidSteerNode()
    rclpy.spin(skid_steer_node)
    skid_steer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()