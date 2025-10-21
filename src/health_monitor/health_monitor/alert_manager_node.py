import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interfaces.msg import RobotStatus  
from collections import deque
from time import time

class AlertManagerNode(Node):
    def __init__(self):
        super().__init__('alert_manager_node')

        # Subscribe to the /robot_status topic
        self.subscription = self.create_subscription(String,'health_status', self.alert_callback, 10)
        self.last_log_time = {"HEALTHY": 0.0, "WARNING": 0.0, "CRITICAL": 0.0}
        self.last_status = None
        self.log_interval = 10.0

        self.get_logger().info("Alert Manager Node started and subscribed to health_status")


    def alert_callback(self, msg: String):
        try:
            status, reason = msg.data.split(":", 1)
            status = status.strip().upper()
            reason = reason.strip()
        except ValueError:
            status, reason = msg.data.strip().upper(), "No reason provided"

        now = time()

        if status == self.last_status and now - self.last_log_time.get(status, 0.0) < self.log_interval:
            return
            
        if status == "HEALTHY":
            self.get_logger().info(f"[alert_manager]: {reason}")
        elif status == "WARNING":
            self.get_logger().warn(f"[alert_manager]: {reason}")
        elif status == "CRITICAL":
            self.get_logger().error(f"[alert_manager]: CRITICAL ALERT - {reason}")
        else:
            self.get_logger().warn(f"[alert_manager]: Unknown status: '{status}' ({reason})")

        self.last_log_time[status] = now
        self.last_status = status

def main(args=None):
    rclpy.init(args=args)
    node = AlertManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()