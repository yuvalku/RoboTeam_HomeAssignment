import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from utilities.kinematics import DifferentailDrive, load_settings

class SkidSteerNode(Node):
    def __init__(self):
        super().__init__('skid_steer_node')
        
        self.num_wheels, self.wheel_radius, self.wheel_separation = load_settings('settings.xml')
        self.dd_controller = DifferentailDrive(self.wheel_radius, self.wheel_separation, self.num_wheels)

        self.cmd_vel_sub = self.create_subscription(
            Twist, 'skid_vel_cmd', self.cmd_vel_callback, 10)
        #self.odom_pub = self.create_publisher(
            #Odometry, 'odom', 10)

        self.odom_msg = Odometry()
        self.timer = self.create_timer(0.1, self.publish_odometry)

    def cmd_vel_callback(self, msg: Twist):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        left_wheel_velocity, right_wheel_velocity = self.dd_controller.calculate_wheels_velocities(linear_velocity, angular_velocity)
        self.get_logger().info(f'Left Wheel Velocity: {left_wheel_velocity}, Right Wheel Velocity: {right_wheel_velocity}')

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