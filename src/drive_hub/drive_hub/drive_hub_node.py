import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from interfaces.msg import MpcInput, MotorsVelocity, CanFrame

STOP_DATA = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00]

class DriveHub(Node):
    def __init__(self):
        super().__init__('drive_hub')
        self.create_subscriptions()
        self.create_publishers()
        self.mpc_cmd_vel = None
        self.mpc_imu = None
        self.timer = self.create_timer(0.5, self.run_mpc_routine)
        self.timer_canceled = False

    def create_subscriptions(self):
        self.create_subscription(Twist, 'cmd_vel_input', self.cmd_vel_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Twist, 'mpc_output', self.on_mpc_output, 10)
        self.create_subscription(CanFrame, 'can_status', self.get_can_status, 10)

    def create_publishers(self):
        self.mpc_input_publisher = self.create_publisher(MpcInput, 'mpc_input', 10)
        self.steer_publisher = self.create_publisher(Twist, 'skid_vel_cmd', 10)

    def cmd_vel_callback(self, msg: Twist):
        self.mpc_cmd_vel = msg
        self.get_logger().info(f'Received command velocity: {msg}')

    def imu_callback(self, msg: Imu):
        self.mpc_imu = msg

    def run_mpc_routine(self):
        if self.mpc_cmd_vel and self.mpc_imu:
            mpc_input = MpcInput()
            mpc_input.cmd_vel = self.mpc_cmd_vel
            mpc_input.imu = self.mpc_imu
            self.mpc_input_publisher.publish(mpc_input)

    def on_mpc_output(self, msg: Twist):
        steer_msg = MotorsVelocity()
        steer_msg = msg
        self.steer_publisher.publish(steer_msg)
        self.get_logger().info(f'Published steer data: {steer_msg}')

    def get_can_status(self, msg: CanFrame):
        hex_data = [hex(byte) for byte in msg.data]
        if msg.id in [0x103, 0x104] and msg.data[3] != 0x00:
            self.get_logger().info(f'Received CAN message: ID={hex(msg.id)}, Data={hex_data}')
            self.stop()
        else: 
            if self.timer_canceled:
                self.timer = self.create_timer(0.5, self.run_mpc_routine) 
        

    def stop(self):
        self.timer.cancel()
        self.timer_canceled = True
        self.publish_final_commands()

    def publish_final_commands(self):
        self.publish_can_message(0x203, STOP_DATA)  # right motor
        self.publish_can_message(0x204, STOP_DATA)  # left motor

    def publish_can_message(self, motor_id, data):
        can_frame = CanFrame()
        can_frame.id = motor_id
        can_frame.data = data
        self.steer_publisher.publish(can_frame)


def main(args=None):
    rclpy.init(args=args)
    drive_hub = DriveHub()
    rclpy.spin(drive_hub)
    drive_hub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()