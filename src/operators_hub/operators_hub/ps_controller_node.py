import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class XBoxJoystickController(Node):
    def __init__(self):
        super().__init__('xbox_joystick_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_input', 10)
        self.image_sub = self.create_subscription(Image, 'video_frames', self.image_callback, 10)

        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        self.bridge = CvBridge()
        self.display_video = False
        self.e_stop_active = False
        self.current_frame = None
        self.timer = self.create_timer(0.02, self.read_joystick) 
        #self.display_timer = self.create_timer(0.03, self.display_frame)
        #self.get_logger().info(f'Joystick initialized: {self.joystick.get_name()}')

    def read_joystick(self):
        pygame.event.pump()

        if self.joystick.get_button(1):
            self.e_stop_active = not self.e_stop_active
            self.get_logger().info(f"E-stop {'activated' if self.e_stop_active else 'deactivated'}.")
            pygame.time.wait(300)
        if self.e_stop_active:
            # Publish zero velocity when e-stop is active
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

        linear_velocity = self.joystick.get_axis(1)
        angular_velocity = self.joystick.get_axis(0)
        linear_velocity = -linear_velocity * 250.0
        angular_velocity = angular_velocity * 250.0
        if linear_velocity < 15.0 and linear_velocity > -15.0:
            linear_velocity = 0.0
        #self.get_logger().info(f'Liner Velocity: {linear_velocity}')
        self.publish_velocity(linear_velocity, angular_velocity)
        

    def publish_velocity(self, linear_velocity, angular_velocity):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_velocity
        cmd_vel_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def start_video_subscription(self):
        self.image_sub = self.create_subscription(
            Image, 'video_frames', self.display_image, 10
        )

    def stop_video_subscription(self):
        if self.image_sub:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
            cv2.destroyAllWindows()

    def display_image(self, msg):
        if self.current_frame is not None:
            # Display the current frame using OpenCV
            cv2.imshow("Video Stream", self.current_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):  # Exit on pressing 'q'
                self.get_logger().info("Exiting video display...")
                rclpy.shutdown()       

def main(args=None):
    rclpy.init(args=args)
    joystick_controller = XBoxJoystickController()
    rclpy.spin(joystick_controller)
    joystick_controller.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__ == "__main__":
    main()