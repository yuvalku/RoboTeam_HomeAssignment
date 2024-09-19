import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np
from interfaces.msg import mpc_input, mpc_output

class DriveHub(Node):

    def __init__(self):
        super().__init__('twist_mux_node')
        self.drive_cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.mpc_sub = self.create_subscription(
            mpc_output, 'mpc_input', self.odom_callback, 10)
        self.mpc_input_cmd = self.create_publisher(
            mpc_input, 'mpc_output', 10)
        self.mpc_cmd_vel = None
        self.odom = None


    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw.
        quaternion = [x, y, z, w]
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
    def cmd_vel_callback(self, msg: Twist):
        self.mpc_cmd_vel = msg

    def odom_callback(self, msg: Odometry):
        self.odom = msg
    
    def mpc_routine(self):
        mpc_input_msg = mpc_input()
        self.mpc_input_cmd.publish(mpc_input_msg)

