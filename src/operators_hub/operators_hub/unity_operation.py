import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from infrastructure.sharedMQ import ZMQModule

class UnityTeleop(Node):
    def __init__(self):
        super().__init__('unity_teleop')
        self.publisher = self.create_publisher(Twist, 'cmd_vel_input', 10)
        self.current_speed = 100.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Initialize ZMQ module
        self.zmq_module = ZMQModule(host="0.0.0.0", port_out=5555, port_in=5556)
        self.get_logger().info("ZMQ module initialized.")

        # Timer to process incoming messages and publish odometry/status
        self.timer = self.create_timer(0.1, self.process_message)

    def process_message(self):
        """
        Periodically checks for new ZMQ messages and processes them.
        """
        message = self.zmq_module.get_message()
        if message:
            self.get_logger().info(f"Received ZMQ message: {message}")
            self.handle_data(message)
            self.publish_cmd()

        # Send odometry and status (example)
        odometry = {"x": 1.0, "y": 2.0, "theta": 0.5}  # Example odometry data
        self.zmq_module.send_message("odometry", str(odometry))

    def handle_data(self, message):
        """
        Handles directional commands and updates linear/angular velocities.
        """
        if message in ['forward', 'backward', 'left', 'right']:
            if message == 'forward':
                self.linear_velocity = self.current_speed
                self.angular_velocity = 0.0
            elif message == 'backward':
                self.linear_velocity = -self.current_speed
                self.angular_velocity = 0.0
            elif message == 'left':
                self.linear_velocity = 0.0
                self.angular_velocity = self.current_speed
            elif message == 'right':
                self.linear_velocity = 0.0
                self.angular_velocity = -self.current_speed
        # else:
        #     self.get_logger().warning(f"Unknown command received: {message}")

    def publish_cmd(self):
        """
        Publishes the current velocities as a Twist message.
        """
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.angular.z = self.angular_velocity
        self.publisher.publish(twist)

    def destroy_node(self):
        """
        Cleanup resources on node destruction.
        """
        self.zmq_module.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    unity_teleop = UnityTeleop()
    try:
        rclpy.spin(unity_teleop)
    except KeyboardInterrupt:
        pass
    finally:
        unity_teleop.destroy_node()
        rclpy.shutdown()
