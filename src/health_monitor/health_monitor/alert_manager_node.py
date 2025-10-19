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

        self.log_interval = 1.0

        self.get_logger().info("Alert Manager Node started and subscribed to /health_status")


    def alert_callback(self, msg: String):
        now = time()

        if now - self.last_log_time.get(status, 0) >= self.log_interval:
            if status == "HEALTHY":
                self.get_logger().info("[alert_manager]: System is HEALTHY ")
            elif status == "WARNING":
                self.get_logger().warn("[alert_manager]: System in WARNING")
            elif status == "CRITICAL":
                self.get_logger().error("[alert_manager]: SYSTEM CRITICAL Immediate attention required")
            else:
                self.get_logger().info(f"[alert_manager]: Unknown status '{status}'")

            self.last_log_time[status] = now

def main(args=None):
    rclpy.init(args=args)
    node = AlertManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()