# ps_joystick_controller.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import pygame
from interfaces.msg import Image420, ManipulatorControl
import cv2
from cv_bridge import CvBridge
from interfaces.srv import GetCameraIp
from cameras.camera_manager import CameraManager
import numpy as np

class PSJoystickController(Node):
    def __init__(self):
        super().__init__('ps_joystick_controller')
        self.qos_video_profile = QoSProfile(depth=5,
                                            reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                            durability=QoSDurabilityPolicy.VOLATILE)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_input', 10)
        self.camera_ip_pub = self.create_publisher(String, 'camera_ip', 10)
        self.ip_service = self.create_service(GetCameraIp, 'get_camera_ip', self.handle_camera_ip_request)
        self.manipulator_control_pub = self.create_publisher(ManipulatorControl, 'manipulator', 10)

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
        
        self.joint_names = ["pan", "shoulder", "elbow1", "elbow2", "wrist", "gripper"]
        self.current_joint_index = 0
        self.manipulator_mode = False
        self.last_hat_x = 0
        self.last_lb_state = False

        self.speed_levels = ["low", "medium", "fast"]
        self.current_speed_level = 1  # Start at medium
        self.last_rb_state = False
        self.joint_max_speeds = {
            "pan": 1000,
            "shoulder": 1000,
            "elbow1": 1500,
            "elbow2": 150,
            "wrist": 150,
            "gripper": 500,  # Arbitrary for gripper, if you want
        }



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

        lb_state = self.joystick.get_button(6)
        rb_state = self.joystick.get_button(7)

        if lb_state and not self.last_lb_state:
            self.manipulator_mode = not self.manipulator_mode
            self.get_logger().info(f"Manipulator mode {'enabled' if self.manipulator_mode else 'disabled'}.")
            pygame.time.wait(300)
        self.last_lb_state = lb_state

        if rb_state and not self.last_rb_state:
            self.current_speed_level = (self.current_speed_level + 1) % len(self.speed_levels)
            self.get_logger().info(f"Speed mode changed to: {self.speed_levels[self.current_speed_level]}")
            pygame.time.wait(300)
        self.last_rb_state = rb_state
    
        if self.manipulator_mode:
            hat_x, hat_y = self.joystick.get_hat(0)
            if hat_x != self.last_hat_x:
                if hat_x == 1:
                    self.current_joint_index = (self.current_joint_index + 1) % len(self.joint_names)
                    self.get_logger().info(f"Joint selected: {self.joint_names[self.current_joint_index]}")
                    pygame.time.wait(200)
                elif hat_x == -1:
                    self.current_joint_index = (self.current_joint_index - 1) % len(self.joint_names)
                    self.get_logger().info(f"Joint selected: {self.joint_names[self.current_joint_index]}")
                    pygame.time.wait(200)
            self.last_hat_x = hat_x
            base_speed = 0
            if hat_y == 1:  # UP
                base_speed = 1
            elif hat_y == -1:  # DOWN
                base_speed = -1
            else:
                base_speed = 0

            joint_name = self.joint_names[self.current_joint_index]
            max_speed = self.joint_max_speeds.get(joint_name, 500)  # Default fallback
            if self.speed_levels[self.current_speed_level] == "low":
                speed = int(max_speed * 0.2)
            elif self.speed_levels[self.current_speed_level] == "medium":
                speed = int(max_speed * 0.5)
            else:  # fast
                speed = max_speed

            joint_speed = base_speed * speed

            msg = ManipulatorControl()
            msg.joint_name = joint_name
            msg.joint_speed = joint_speed
            msg.is_manual = True
            self.manipulator_control_pub.publish(msg)

            if base_speed != 0:  # Only log if moving
                self.get_logger().info(f"Manipulator control: {joint_name} moving at {joint_speed} (speed mode {self.speed_levels[self.current_speed_level]})")

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
    joystick_controller = PSJoystickController()
    rclpy.spin(joystick_controller)
    joystick_controller.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__ == "__main__":
    main()
