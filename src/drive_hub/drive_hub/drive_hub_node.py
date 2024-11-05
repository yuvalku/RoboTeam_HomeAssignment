import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import numpy as np
from interfaces.msg import MpcInput, MpcOutput, WheelsVelocity, CanFrame

class DriveHub(Node):

    def __init__(self):
        super().__init__('drive_hub_node')
        self.drive_cmd_sub = self.create_subscription(
            WheelsVelocity, 'cmd_vel_input', self.cmd_vel_callback, 10)
        self.odom_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
        self.mpc_input_cmd = self.create_publisher(
            MpcInput, 'mpc_input', 10)
        self.steer_pub = self.create_publisher(
            Twist, 'skid_vel_cmd', 10)
        self.mpc_output_sub = self.create_subscription(
            MpcOutput, 'mpc_output', self.steer_callback, 10)
        self.mpc_routine = self.create_timer(0.5, self.mpc_routine)
        self.mpc_cmd_vel = None
        self.mpc_imu = None

    def cmd_vel_callback(self, msg: Twist):
        self.mpc_cmd_vel = msg

    def imu_callback(self, msg: Imu):
        self.mpc_imu = msg

    def mpc_routine(self):
        if self.mpc_cmd_vel is not None and self.mpc_imu is not None:
            mpc_input_msg = MpcInput()
            mpc_input_msg.cmd_vel = self.mpc_cmd_vel
            mpc_input_msg.imu = self.mpc_imu
            self.mpc_input_cmd.publish(mpc_input_msg)
            #self.get_logger().info("publishing input data to mpc: {}".format(mpc_input_msg))

    def steer_callback(self, msg: MpcOutput):
        steer_msg = WheelsVelocity()
        steer_msg = msg.cmd_vel
        self.steer_pub.publish(steer_msg)

    


def main(args=None):
    rclpy.init(args=args)
    drive_hub = DriveHub()
    rclpy.spin(drive_hub)
    drive_hub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()