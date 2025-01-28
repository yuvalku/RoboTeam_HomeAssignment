import rclpy
from rclpy.node import Node
import cv2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from cv_bridge import CvBridge
from cameras.camera_manager import CameraManager
from infrastructure.sharedMQ import ZMQModule
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from interfaces.srv import GetCameraIp
from std_msgs.msg import String


class UnityTeleop(Node):
    def __init__(self):
        super().__init__('unity_teleop')
        self.publisher = self.create_publisher(Twist, 'cmd_vel_input', 10)
        self.camera_ip_pub = self.create_publisher(String, 'camera_ip_change', 10)
        qos_video_profile = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE)
        self.video_subscriber = self.create_subscription(Image, 'video_frames', self.image_callback, qos_video_profile)
        self.ip_service = self.create_service(GetCameraIp, 'get_camera_ip', self.handle_camera_ip_request)
        self.current_speed = 100.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.bridge = CvBridge()
        self.display_video = False
        self.e_stop_active = False
        self.current_frame = None
        self.camera_manager = CameraManager("ROOK_1171")
        self.current_camera_ip = self.camera_manager.get_initial_camera_ip()
        self.counter = 0
        # Initialize ZMQ module
        self.zmq_module = ZMQModule()
        self.get_logger().info("ZMQ module initialized.")

        # Timer to process incoming messages and publish odometry/status
        self.timer = self.create_timer(0.1, self.process_message)

    def process_message(self):

        message = self.zmq_module.get_message()
        if message:
            self.get_logger().info(f"Received ZMQ message: {message}")
            self.handle_data(message)
            # self.publish_cmd()


    def handle_camera_ip_request(self, request, response):
        response.camera_ip = self.current_camera_ip
        self.get_logger().info(f"Service request handled: returning {self.current_camera_ip}")
        return response
    
    def publish_camera_ip_change(self, new_ip):
        if new_ip != self.current_camera_ip:
            self.current_camera_ip = new_ip
            msg = String()
            msg.data = self.current_camera_ip
            self.camera_ip_pub.publish(msg)
            self.get_logger().info(f"Camera IP updated to: {new_ip}")

    def image_callback(self, msg):
        try:
            # Convert ROS Image to CV2 image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().info(f"Received image from ROS, size: {cv_image.shape}")

            # Encode as JPEG and publish to ZMQ
            _, encoded_image = cv2.imencode('.jpg', cv_image)
            #cv2.imwrite( f"frame_{self.counter}.jpg", cv_image)
            self.zmq_module.send_message("video", encoded_image.tobytes())
            self.get_logger().info(f"Sent image to ZMQ, size: {len(encoded_image.tobytes())}")
            #self.counter += 1
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

        
    def handle_data(self, message):
        """
        Handles directional commands and updates linear/angular velocities.
        """
        if message.topic in ['switch_camera', 'drive']:
            if message == 'switch_camera':
                current_camera = CameraManager.toggle_camera()
                self.publish_camera_ip_change(current_camera)
                self.restart_video_subscription()

        # else:
        #     self.get_logger().warning(f"Unknown command received: {message}")

    def publish_cmd(self):
        """
        Publishes the current velocities as a Twist message.
        """
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.angular.z = self.angular_velocity
        self.publisher.publish(twist)


    def start_video_subscription(self):
        self.image_sub = self.create_subscription(
            Image, 'video_frames', self.display_image, 10
        )

    def stop_video_subscription(self):
        if self.image_sub:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
            cv2.destroyAllWindows()

    def restart_video_subscription(self):
        self.stop_video_subscription()
        self.start_video_subscription()


    def destroy_node(self):
        """
        Cleanup resources on node destruction.
        """
        self.zmq_module.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    unity_teleop = UnityTeleop()
    try:
        rclpy.spin(unity_teleop)
    except KeyboardInterrupt:
        pass
    finally:
        unity_teleop.destroy_node()
        rclpy.shutdown()
