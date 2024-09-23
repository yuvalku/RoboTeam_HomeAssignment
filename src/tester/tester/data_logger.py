import rclpy
from rclpy.node import Node
from interfaces.msg import MpcInput, MpcOutput
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        self.create_subscription(Twist, 'cmd_vel_input', self.cmd_vel_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(MpcOutput, 'mpc_output', self.mpc_output_callback, 10)

    def cmd_vel_callback(self, msg: Twist):
        self.get_logger().info('Received cmd_vel: {}\n'.format(msg))

    def imu_callback(self, msg: Imu):
        self.get_logger().info('Received IMU data: {}\n'.format(msg))

    def mpc_output_callback(self, msg: MpcOutput):
        self.get_logger().info('Received MPC output: {}\n'.format(msg))

def main(args=None):
    rclpy.init(args=args)
    data_logger = DataLogger()
    rclpy.spin(data_logger)
    data_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()