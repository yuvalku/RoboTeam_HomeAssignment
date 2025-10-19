import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interfaces.msg import RobotStatus  
from collections import deque
from time import time

class HealthAnalyzerNode(Node):
    def __init__(self):
        super().__init__('health_analyzer_node')

        # Subscribe to the /robot_status topic
        self.subscription = self.create_subscription(RobotStatus,'robot_status', self.status_callback, 10)


        self.pub = self.create_publisher(String, 'health_status', 10)
        self.timer = self.create_timer(0.2, self.health_classification)

        self.status_buffer = deque(maxlen=10)
        self.last_critical_time = None

        self.get_logger().info('Health Analyzer Node started')


    
    def status_callback(self, msg: RobotStatus):
        self.status_buffer.append(msg)


    def health_classification(self):
        if not self.status_buffer:
            return

        latest = self.status_buffer[-1]

        #Current speed of the left wheel motor (in RPM)
        rpm_left = latest.left_rpm
        #Current speed of the right wheel motor (in RPM)
        rpm_right = latest.right_rpm
        battery = latest.battery_charge
        bit_error = (latest.left_bit_error != 0 or latest.right_bit_error != 0 or latest.battery_bit_error != 0)   

        classification  = self.determine_classification(rpm_left, rpm_right, battery, bit_error)   

        msg = String()
        msg.data = classification
        self.pub.publish(msg)
        self.get_logger().info(f"[health_analyzer]: {classification }")



    def determine_classification(self, rpm_left, rpm_right, battery, bit_error):

        if battery < 15:
            return "CRITICAL"

        if rpm_left == 0 and rpm_right == 0:
            if self.last_critical_time is None:
                self.last_critical_time = time()
            elif time() - self.last_critical_time > 5:
                return "CRITICAL"
        else:
            self.last_critical_time = None

        if 15 < battery < 25 or abs(rpm_left) > 2000 or abs(rpm_right) > 2000 or bit_error:
            return "WARNING"

        return "HEALTHY"



def main(args=None):
    rclpy.init(args=args)
    node = HealthAnalyzerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()