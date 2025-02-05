# xbox_joystick_controller.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import pygame
from interfaces.msg import Image420
import cv2
from cv_bridge import CvBridge
from interfaces.srv import GetCameraIp
from cameras.camera_manager import CameraManager
import numpy as np

class XBoxJoystickController(Node):
    def __init__(self):
        super().__init__('xbox_joystick_controller')
        self.qos_video_profile = QoSProfile(depth=5,
                                            reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                            durability=QoSDurabilityPolicy.VOLATILE)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_input', 10)
        self.camera_ip_pub = self.create_publisher(String, 'camera_ip', 10)
        self.ip_service = self.create_service(GetCameraIp, 'get_camera_ip', self.handle_camera_ip_request)

        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        self.bridge = CvBridge()
        self.display_video = False
        self.e_stop_active = False
        self.timer = self.create_timer(0.03, self.read_joystick) 
        self.camera_manager = CameraManager("ROOK_1171")
        self.current_camera_ip = self.camera_manager.get_initial_camera_ip()
        self.publish_camera_ip(self.current_camera_ip)

    def handle_camera_ip_request(self, request, response):
        response.camera_ip = self.current_camera_ip
        self.get_logger().info(f"Service request handled: returning {self.current_camera_ip}")
        return response
    
    def read_joystick(self):
        pygame.event.pump()

        if self.joystick.get_button(1):
            self.e_stop_active = not self.e_stop_active
            self.get_logger().info(f"E-stop {'activated' if self.e_stop_active else 'deactivated'}.")
            pygame.time.wait(300)
        if self.e_stop_active:
            self.publish_velocity(0.0, 0.0)
            return
        if self.joystick.get_button(0):
            self.display_video = not self.display_video
            self.get_logger().info(f"Video display {'enabled' if self.display_video else 'disabled'}.")
            if self.display_video:
                self.start_video_subscription()
            else:
                self.stop_video_subscription()
            pygame.time.wait(300)
        
        if self.joystick.get_button(2) and self.display_video:
            self.current_camera = self.camera_manager.toggle_camera()
            self.publish_camera_ip(self.current_camera)
            self.restart_video_subscription()
            pygame.time.wait(300)

        linear_velocity = -self.joystick.get_axis(1) * 250.0
        angular_velocity = self.joystick.get_axis(0) * 250.0
        if abs(linear_velocity) < 15.0:
            linear_velocity = 0.0
        self.publish_velocity(linear_velocity, angular_velocity)
        
    def publish_camera_ip(self, ip):
        msg = String()
        msg.data = ip
        self.camera_ip_pub.publish(msg)
        self.get_logger().info(f"Published new camera IP: {ip}")

    def publish_velocity(self, linear_velocity, angular_velocity):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_velocity
        cmd_vel_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def start_video_subscription(self):
        self.image_sub = self.create_subscription(
            Image420, 'video_frames', self.display_image, self.qos_video_profile
        )

    def stop_video_subscription(self):
        if hasattr(self, 'image_sub') and self.image_sub:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
            cv2.destroyAllWindows()

    def restart_video_subscription(self):
        self.stop_video_subscription()
        self.start_video_subscription()
        
    def display_image(self, msg):
        try:
            # Convert the incoming byte array to a NumPy array and decode the JPEG.
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

def main(args=None):
    rclpy.init(args=args)
    joystick_controller = XBoxJoystickController()
    rclpy.spin(joystick_controller)
    joystick_controller.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__ == "__main__":
    main()
