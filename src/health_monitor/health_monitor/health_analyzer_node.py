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

        classification, reason  = self.determine_classification(rpm_left, rpm_right, battery, bit_error)   

        msg = String()
        msg.data = f"{status}:{reason}" 
        self.pub.publish(msg)

        # Log in analyzer
        self.get_logger().info(f"[health_analyzer]: Robot status: {status} - {reason}")



    def determine_classification(self, rpm_left, rpm_right, battery, bit_error):

        if battery < 15:
            return "CRITICAL", f"Low battery - Charge level: {battery}%"

        if rpm_left == 0 and rpm_right == 0:
            if self.last_critical_time is None:
                self.last_critical_time = time()
            elif time() - self.last_critical_time > 5:
                return "CRITICAL", "Both motors unresponsive for 6 seconds"
        else:
            self.last_critical_time = None

        if 15 <= battery <= 25:
            return "WARNING", f"Battery warning - Charge level: {battery}%"
        if abs(rpm_left) > 2000 or abs(rpm_right) > 2000:
            return "WARNING", f"Motor RPM out of range (L={rpm_left}, R={rpm_right})"
        if bit_error:
            return "WARNING", "BIT error detected"

        # --- Healthy ---
        return "HEALTHY", "All systems nominal"


def main(args=None):
    rclpy.init(args=args)
    node = HealthAnalyzerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()