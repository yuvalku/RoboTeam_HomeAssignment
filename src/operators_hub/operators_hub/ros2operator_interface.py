import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from interfaces.msg import Image420, ManipulatorControl
from interfaces.srv import GetCameraIp
import cv2
from cv_bridge import CvBridge
from cv_bridge import CvBridge
from cameras.camera_manager import CameraManager
import numpy as np

class ROS2OperatorInterface(Node):
    def __init__(self):
        super().__init__('ros2_operator_interface')
        self.qos_video_profile = rclpy.qos.QoSProfile(depth=5,
                                                        reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
                                                        durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_input', 10)
        self.camera_ip_pub = self.create_publisher(String, 'camera_ip', 10)
        self.manipulator_control_pub = self.create_publisher(ManipulatorControl, 'manipulator', 10)
        self.ip_service = self.create_service(GetCameraIp, 'get_camera_ip', self.handle_camera_ip_request)

        self.declare_parameter('camera_name', 'TIGR_907')  # default value
        camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.camera_manager = CameraManager(camera_name)
        self.current_camera_ip = self.camera_manager.get_initial_camera_ip()
        self.camera_manager.current_camera = self.current_camera_ip
        self.publish_camera_ip(self.current_camera_ip)
        self.bridge = CvBridge()
        self.image_sub = None

    def handle_camera_ip_request(self, request, response):
        response.camera_ip = self.current_camera_ip
        self.get_logger().info(f"Service request handled: returning {self.current_camera_ip}")        
        return response
    
    def publish_camera_ip(self, camera_ip):
        msg = String()
        msg.data = camera_ip
        self.camera_ip_pub.publish(msg)
        self.get_logger().info(f"Publishing camera IP: {camera_ip}")

    def publish_velocity(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"Publishing velocity: {linear_x}, {angular_z}")

    def publish_manipulator_control(self, joint_name, position=0, speed=0, is_manual=True):
        manipulator_control = ManipulatorControl()
        manipulator_control.joint_name = joint_name
        manipulator_control.joint_position = position
        manipulator_control.joint_speed = speed
        manipulator_control.is_manual = is_manual
        self.manipulator_control_pub.publish(manipulator_control)
        self.get_logger().info(f"Publishing manipulator control: {joint_name}, {position}, {speed}, {is_manual}")

    def start_video_subscription(self):
        if not self.image_sub:
            self.image_sub = self.create_subscription(
                Image420, 'video_frames', self.display_image, self.qos_video_profile)
            
    def stop_video_subscription(self):
        if self.image_sub:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
            cv2.destroyAllWindows()

    def restart_video_subscription(self):
        self.stop_video_subscription()
        self.start_video_subscription()

    def display_image(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is not None:
                cv2.imshow("Video Stream", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.get_logger().info("Exiting video display...")
                    rclpy.shutdown()
            else:
                self.get_logger().warning("Failed to decode frame.")
        except Exception as e:
            self.get_logger().error(f"Error displaying image: {e}")