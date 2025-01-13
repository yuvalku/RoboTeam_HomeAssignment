#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# Import your RobotStatus message
from interfaces.msg import RobotStatus

class RookStatusDisplayNode(Node):
    def __init__(self):
        super().__init__('rook_status_display_node')

        # Subscribe to the /robot_status topic
        self.subscription = self.create_subscription(
            RobotStatus,
            'robot_status',
            self.status_callback,
            10
        )

        # Prevent unused variable warning
        self.subscription

        self.get_logger().info("RookStatusDisplayNode started, listening to /robot_status...")

    def status_callback(self, msg: RobotStatus):
        """
        Called whenever a new RobotStatus is published.
        We'll extract the battery charge, left_rpm, right_rpm,
        and display them in the console.
        """
        left_rpm = msg.left_rpm
        right_rpm = msg.right_rpm
        battery_charge = msg.battery_charge  # 0..100

        # Build a simple ASCII progress bar for battery
        bar_length = 20  # length of the bar in characters
        filled_len = (battery_charge * bar_length) // 100
        bar_str = '=' * filled_len + '-' * (bar_length - filled_len)

        # Print or log the info
        # Here we do a console print for a "visual" effect
        # If you want it to be more dynamic, you might consider clearing the screen, etc.
        print("\n" + "-" * 40)
        print("          Rook Status Display         ")
        print("-" * 40)
        print(f"Left RPM:  {left_rpm}")
        print(f"Right RPM: {right_rpm}")
        print(f"Battery:   [{bar_str}] {battery_charge}%")
        print("-" * 40 + "\n")

        # Alternatively, you can use self.get_logger().info(...) if you prefer logs
        # self.get_logger().info(...)
        # But printing to console might be more visual for this example.

def main(args=None):
    rclpy.init(args=args)
    node = RookStatusDisplayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
