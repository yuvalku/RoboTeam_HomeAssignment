#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interfaces.msg import RobotStatus
from collections import deque
import time


class HealthAnalyzerNode(Node):
    def __init__(self):
        super().__init__('health_analyzer_node')

        # --- Parameters ---
        self.declare_parameter('battery_critical_threshold', 15)
        self.declare_parameter('battery_warning_threshold', 25)
        self.declare_parameter('rpm_limit', 2000)
        self.declare_parameter('rpm_timeout', 5.0)

        self.battery_critical_threshold = self.get_parameter('battery_critical_threshold').value
        self.battery_warning_threshold = self.get_parameter('battery_warning_threshold').value
        self.rpm_limit = self.get_parameter('rpm_limit').value
        self.rpm_timeout = self.get_parameter('rpm_timeout').value

        # --- Communication setup ---
        self.subscription = self.create_subscription(RobotStatus, '/robot_status', self.robot_status_callback, 10)
        self.publisher = self.create_publisher(String, '/health_status', 10)

        # --- State ---
        self.rolling_window = deque(maxlen=10)
        self.last_rpm_time = time.time()

        # --- Timer at 0.2 sec ---
        self.timer = self.create_timer(0.2, self.publish_health_status)

        self.get_logger().info("Alert Manager Node started and subscribed to health_status")
        

    def robot_status_callback(self, msg: RobotStatus):
        try:
            self.rolling_window.append(msg)
            # update last active rpm time if any wheel is moving
            if msg.left_rpm != 0 or msg.right_rpm != 0:
                self.last_rpm_time = time.time()
        except Exception as e:
            self.get_logger().error(f"Error processing RobotStatus: {e}")

    def classify_health(self, msg: RobotStatus) -> str:
        try:
            left_rpm = msg.left_rpm
            right_rpm = msg.right_rpm
            battery = msg.battery_charge
            bit_error = (msg.left_bit_error or msg.right_bit_error or msg.battery_bit_error)

        except AttributeError as e:
            self.get_logger().error(f"Invalid RobotStatus format: {e}")
            return "CRITICAL: Invalid message format"

        time_since_rpm = time.time() - self.last_rpm_time

        if battery < self.battery_critical_threshold:
            return "CRITICAL: Battery critically low"
        if left_rpm == 0 and right_rpm == 0 and time_since_rpm > self.rpm_timeout:
            return f"CRITICAL: Both motors unresponsive for {round(time_since_rpm,1)}s"

        if battery <= self.battery_warning_threshold:
            return f"WARNING: Battery warning - charge level: {battery}%"
        if bit_error:
            return "WARNING: BIT error detected"
        if abs(left_rpm) > self.rpm_limit or abs(right_rpm) > self.rpm_limit:
            return f"WARNING: RPM out of range (L={left_rpm}, R={right_rpm})"

        # --- HEALTHY ---
        return "HEALTHY: All systems nominal"

    def publish_health_status(self):
        if not self.rolling_window:
            return

        latest_msg = self.rolling_window[-1]
        status_text = self.classify_health(latest_msg)

        msg = String()
        msg.data = status_text
        self.publisher.publish(msg)

        self.get_logger().info(f"[health_analyzer] {status_text}")


def main(args=None):
    rclpy.init(args=args)
    node = HealthAnalyzerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
