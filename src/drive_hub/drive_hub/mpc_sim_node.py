import rclpy
from rclpy.node import Node
from interfaces.msg import MpcInput, MpcOutput

class MPCSimulator(Node):
    def __init__(self):
        super().__init__('mpc_simulator')
        self.mpc_sub = self.create_subscription(MpcInput, 'mpc_input', self.mpc_callback, 10)
        self.mpc_pub = self.create_publisher(MpcOutput, 'mpc_output', 10)

    def mpc_callback(self, msg: MpcInput):
        # Manipulate velocity data
        output_msg = MpcOutput()
        output_msg.cmd_vel = msg.cmd_vel
        output_msg.cmd_vel.linear.x *= 2  # Double the linear velocity
        output_msg.cmd_vel.linear.y *= 2
        output_msg.cmd_vel.angular.z *= 2
        self.mpc_pub.publish(output_msg)
        self.get_logger().info('Publishing MPC output: {}'.format(output_msg))

def main(args=None):
    rclpy.init(args=args)
    mpc_simulator = MPCSimulator()
    rclpy.spin(mpc_simulator)
    mpc_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()