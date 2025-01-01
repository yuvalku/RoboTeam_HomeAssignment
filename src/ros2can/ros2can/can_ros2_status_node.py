import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from interfaces.msg import RobotStatus
from canbridge.can_bridge import CanBridge
from collections import deque


class FieldFilter:
    def __init__(self,maxlen=10):
        self.buffer = deque(maxlen=maxlen)
        self.last_non_zero = 0

    def update(self,new_val: int):
        self.buffer.append(new_val)
        if new_val != 0:
            self.last_non_zero = new_val

    def get_value(self) -> int:
        if len(self.buffer) == 0:
            return 0
        if all(sample ==0 for sample in self.buffer):
            return 0
        else:
            return self.last_non_zero
        
    
class CanRos2Status(Node):
    def __init__(self):
        super().__init__('can_ros2_status_node')

        self.bridge = CanBridge()

        # FieldFilters to smooth or filter out transient zeros
        self.left_rpm_filter = FieldFilter()
        self.right_rpm_filter = FieldFilter()
        self.left_load_filter = FieldFilter()
        self.right_load_filter = FieldFilter()
        self.left_current_filter = FieldFilter()
        self.right_current_filter = FieldFilter()
        self.left_odom_filter = FieldFilter()
        self.right_odom_filter = FieldFilter()
        self.roll_filter = FieldFilter()
        self.pitch_filter = FieldFilter()
        self.yaw_filter = FieldFilter()
        self.battery_current_filter = FieldFilter()
        self.battery_voltage_filter = FieldFilter()
        self.battery_charge_filter = FieldFilter()
        self.left_temp_filter = FieldFilter()
        self.right_temp_filter = FieldFilter()
        self.left_heat_filter = FieldFilter()
        self.right_heat_filter = FieldFilter()

        self.left_bit_error = 0
        self.right_bit_error = 0
        self.battery_bit_error = 0

        self.battery_id = 0

        self.status_publisher = self.create_publisher(RobotStatus, 'robot_status', 10)
        self.read_timer = self.create_timer(0.01, self.read_can_frames)
        self.publish_timer  = self.create_timer(0.1, self.publish_status)
        self.counter = 0

    def read_can_frames(self):
        while True:
            frame = self.bridge.read_one_frame(timeout=0.0)
            if not frame:
                break
            can_id, data = frame
            #self.get_logger().info(f"Received CAN frame: {can_id}, {data}")
            parsed_fields = self.bridge.parse_incoming_frame(can_id, data)
            self.update_fields(parsed_fields)
        
    def update_fields(self, fields):
        """
        Use the dictionary from parse_incoming_frame
        to update each FieldFilter or direct variable.
        """
        #self.get_logger().info(f"fields {fields}")
        if 'left_rpm' in fields:
            self.left_rpm_filter.update(fields['left_rpm'])
        if 'right_rpm' in fields:
            self.right_rpm_filter.update(fields['right_rpm'])

        if 'left_load' in fields:
            self.left_load_filter.update(fields['left_load'])
        if 'right_load' in fields:
            self.right_load_filter.update(fields['right_load'])

        if 'left_current' in fields:
            self.left_current_filter.update(fields['left_current'])
        if 'right_current' in fields:
            self.right_current_filter.update(fields['right_current'])

        if 'left_odom' in fields:
            self.left_odom_filter.update(fields['left_odom'])
        if 'right_odom' in fields:
            self.right_odom_filter.update(fields['right_odom'])

        if 'roll_deg' in fields:
            self.roll_filter.update(fields['roll_deg'])
        if 'pitch_deg' in fields:
            self.pitch_filter.update(fields['pitch_deg'])
        if 'yaw_deg' in fields:
            self.yaw_filter.update(fields['yaw_deg'])

        if 'battery_current' in fields:
            self.battery_current_filter.update(fields['battery_current'])
        if 'battery_voltage' in fields:
            self.battery_voltage_filter.update(fields['battery_voltage'])
        if 'battery_charge' in fields:
            self.battery_charge_filter.update(fields['battery_charge'])

        if 'left_temp' in fields:
            self.left_temp_filter.update(fields['left_temp'])
        if 'right_temp' in fields:
            self.right_temp_filter.update(fields['right_temp'])
        if 'left_heat' in fields:
            self.left_heat_filter.update(fields['left_heat'])
        if 'right_heat' in fields:
            self.right_heat_filter.update(fields['right_heat'])

        if 'left_bit_error' in fields:
            self.left_bit_error = fields['left_bit_error']
        if 'right_bit_error' in fields:
            self.right_bit_error = fields['right_bit_error']
        if 'battery_bit_error' in fields:
            self.battery_bit_error = fields['battery_bit_error']


       
    def publish_status(self):
        """
        Publish a RobotStatus message at 10 Hz, using the most recent data
        from FieldFilters.
        """
        msg = RobotStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"robot_status_{self.counter}"
        self.counter += 1

        msg.left_rpm = self.left_rpm_filter.get_value()
        msg.right_rpm = self.right_rpm_filter.get_value()
        msg.left_load = self.left_load_filter.get_value()
        msg.right_load = self.right_load_filter.get_value()
        msg.left_current = self.left_current_filter.get_value()
        msg.right_current = self.right_current_filter.get_value()
        msg.left_odom = self.left_odom_filter.get_value()
        msg.right_odom = self.right_odom_filter.get_value()
        msg.roll_deg = self.roll_filter.get_value()
        msg.pitch_deg = self.pitch_filter.get_value()
        msg.yaw_deg = self.yaw_filter.get_value()
        msg.battery_current = self.battery_current_filter.get_value()
        msg.battery_voltage = self.battery_voltage_filter.get_value()
        msg.battery_charge = self.battery_charge_filter.get_value()
        msg.left_temp = self.left_temp_filter.get_value()
        msg.right_temp = self.right_temp_filter.get_value()
        msg.left_heat = self.left_heat_filter.get_value()
        msg.right_heat = self.right_heat_filter.get_value()
        msg.left_bit_error = self.left_bit_error
        msg.right_bit_error = self.right_bit_error
        msg.battery_bit_error = self.battery_bit_error

        self.status_publisher.publish(msg)
        self.get_logger().info(f"Publishing message: {msg}")


def main(args=None):
    rclpy.init(args=args)
    can_ros2_stat = CanRos2Status()
    rclpy.spin(can_ros2_stat)
    can_ros2_stat.destroy_node()
    rclpy.shutdown()

if __name__== "__main__":
    main()