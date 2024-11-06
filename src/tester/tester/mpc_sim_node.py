import rclpy
from rclpy.node import Node
from interfaces.msg import MpcInput, MpcOutput
from geometry_msgs.msg import Twist

class MPCSimulator(Node):
    def __init__(self):
        super().__init__('mpc_simulator')
        self.mpc_sub = self.create_subscription(MpcInput, 'mpc_input', self.mpc_callback, 10)
        self.mpc_pub = self.create_publisher(Twist, 'mpc_output', 10)

    def mpc_callback(self, msg: MpcInput):
        # Manipulate velocity data
        output_msg = MpcOutput()
        output_msg = msg.cmd_vel
        output_msg.linear.x *= 1  # Double the linear velocity
        output_msg.linear.y *= 1
        output_msg.angular.z *= 1
        self.mpc_pub.publish(output_msg)
        #self.get_logger().info('Publishing MPC output: {}'.format(output_msg))

def main(args=None):
    rclpy.init(args=args)
    mpc_simulator = MPCSimulator()
    rclpy.spin(mpc_simulator)
    mpc_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()