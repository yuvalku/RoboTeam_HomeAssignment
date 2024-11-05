import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from utilities.kinematics import DifferentailDrive, load_settings
from interfaces.msg import WheelsVelocity

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
        self.wheels_vel_pub = self.create_publisher(
            WheelsVelocity, 'wheels_vel', 10)
        
    def cmd_vel_callback(self, msg: Twist):
        wheelsVelocity = WheelsVelocity()

        wheelsVelocity.left_wheel_velocity, wheelsVelocity.right_wheel_velocity = self.dd_controller.calculate_wheels_velocities(msg.cmd_vel.linear.x, msg.cmd_vel.angular.z)

        wheelsVelocity.left_wheel_direction = -1 # left straight
        wheelsVelocity.right_wheel_direction = 1 # right straight
        if msg.cmd_vel.angular.z < 0: # steer right
            wheelsVelocity.left_wheel_direction = -1 # left straight
            wheelsVelocity.right_wheel_direction = -1 # right backwards
        elif msg.cmd_vel.angular.z > 0: #steer left
            wheelsVelocity.left_wheel_direction = 1 # left backwards
            wheelsVelocity.right_wheel_direction = 1 # right straight
        if msg.cmd_vel.linear.x < 0: # steer back
            wheelsVelocity.left_wheel_direction *= -1 
            wheelsVelocity.right_wheel_direction *= -1

        self.wheels_vel_pub.publish(wheelsVelocity)

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