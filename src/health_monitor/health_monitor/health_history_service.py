#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from collections import deque
from interfaces.srv import GetHealthHistory  # custom service definition

class HealthHistoryService(Node):
    def __init__(self):
        super().__init__('health_history_service')


        self.subscription = self.create_subscription(String, '/health_status', self.health_callback, 10)

        self.srv = self.create_service(GetHealthHistory, 'get_health_history', self.get_history_callback)
        
        self.max_history = 50
        self.history = deque(maxlen=self.max_history)

        self.get_logger().info('Health History Service started successfully')

    def health_callback(self, msg: String):
        self.history.append(msg.data)
        self.get_logger().debug(f"Stored new health status: {msg.data}")

    def get_history_callback(self, request: GetHealthHistory.Request,
                             response: GetHealthHistory.Response):
        if not self.history:
            response.history = []
            self.get_logger().warn('History requested but empty')
            return response

        # Clamp requested count to available entries
        count = min(max(1, request.count), len(self.history))
        response.history = list(self.history)[-count:]
        self.get_logger().info(f"Returned {count} health history entries")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = HealthHistoryService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Health History Service interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
