import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import random
import time

class IMUSimulator(Node):
    def __init__(self):
        super().__init__('imu_simulator')
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.timer = self.create_timer(1.0, self.publish_imu_data)  # Publish every second

    def publish_imu_data(self):
        imu_msg = Imu()
        imu_msg.angular_velocity.x = random.uniform(-1, 1)
        imu_msg.angular_velocity.y = random.uniform(-1, 1)
        imu_msg.angular_velocity.z = random.uniform(-1, 1)
        imu_msg.linear_acceleration.x = random.uniform(-1, 1)
        imu_msg.linear_acceleration.y = random.uniform(-1, 1)
        imu_msg.linear_acceleration.z = random.uniform(-1, 1)
        self.imu_pub.publish(imu_msg)
        #self.get_logger().info('Publishing IMU data: {}'.format(imu_msg))

def main(args=None):
    rclpy.init(args=args)
    imu_simulator = IMUSimulator()
    rclpy.spin(imu_simulator)
    imu_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()