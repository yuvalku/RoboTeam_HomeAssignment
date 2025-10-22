#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class AlertManagerNode(Node):
    def __init__(self):
        super().__init__('alert_manager_node')

        # --- Parameters ---
        self.declare_parameter('alert_cooldown', 10.0)
        self.alert_cooldown = float(self.get_parameter('alert_cooldown').value)

        # --- State tracking ---
        self.last_status = None
        self.last_alert_time = {"HEALTHY": 0.0, "WARNING": 0.0, "CRITICAL": 0.0}

        # --- Subscription ---
        self.subscription = self.create_subscription(String, '/health_status', self.health_status_callback, 10)

        self.get_logger().info(f"Alert Manager Node started and subscribed to health_status")

    def health_status_callback(self, msg: String) -> None:
        now = time.time()
        
        try:
            if ":" in msg.data:
                current_status, reason = msg.data.split(":", 1)
                current_status, reason = current_status.strip().upper(), reason.strip()
            else:
                current_status, reason = msg.data.strip().upper(), ""
        except Exception as e:
            self.get_logger().error(f"Failed to parse /health_status message: {e}")
            return

        last_time = self.last_alert_time.get(current_status, 0.0)
        if current_status == self.last_status and now - last_time < self.alert_cooldown:
            return

        self.log_status(current_status, reason)
        self.last_alert_time[current_status] = now
        self.last_status = current_status


    def log_status(self, status: str, reason: str) -> None:
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())

        if status == "HEALTHY":
            self.get_logger().info("Robot status: HEALTHY - All systems nominal")
        elif status == "WARNING":
            msg = reason or "Warning condition detected"
            self.get_logger().warn(f"[alert_manager] {msg}")
        elif status == "CRITICAL":
            msg = reason or "Critical fault detected"
            self.get_logger().error(f"[alert_manager] CRITICAL ALERT - {msg} [{timestamp}]")
        else:
            self.get_logger().warn(f"[alert_manager] Unknown status '{status}' ({reason})")


def main(args=None):
    rclpy.init(args=args)
    node = AlertManagerNode()
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
