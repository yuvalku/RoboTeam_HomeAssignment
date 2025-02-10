# ps_controller.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from interfaces.msg import Image420
from std_msgs.msg import String, Header
from cv_bridge import CvBridge
from gi.repository import GLib
import threading
from video_hub.gs_pipeline import GStreamerPipeline
from interfaces.srv import GetCameraIp
import cv2
import numpy as np

class VideoStreamNode(Node):
    def __init__(self):
        super().__init__('video_stream_publisher')
        self.pipeline = None
        self.pipeline_thread = None
        self.bridge = CvBridge()
        qos_video_profile = QoSProfile(depth=5,
                                       reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                       durability=QoSDurabilityPolicy.VOLATILE)
        self.image_pub = self.create_publisher(Image420, 'video_frames', qos_video_profile)
        self.camera_ip_sub = self.create_subscription(String, 'camera_ip_change', self.update_camera_ip, 10)
        self.ip_client = self.create_client(GetCameraIp, 'get_camera_ip')
        self.timer = self.create_timer(0.066, self.publish_frame)  # Approximately 15 FPS
        self.current_ip = None
        self.initialized = False
        self.stop_event = threading.Event()

        self.request_initial_ip()

    def request_initial_ip(self):
        while not self.ip_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for 'get_camera_ip' service...")
        try:
            request = GetCameraIp.Request()
            future = self.ip_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result():
                if not self.current_ip:
                    self.current_ip = future.result().camera_ip
                self.get_logger().info(f"Initialized with camera IP: {self.current_ip}")
                self.restart_pipeline(self.current_ip)
                return True
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        return False

    def update_camera_ip(self, msg):
        new_ip = msg.data
        if new_ip != self.current_ip:
            self.current_ip = new_ip
            self.restart_pipeline(new_ip)
            self.get_logger().info(f"Updated pipeline to new IP: {new_ip}")

    def restart_pipeline(self, ip):
        self.stop_pipeline()
        self.pipeline = GStreamerPipeline(ip)
        self.stop_event.clear()
        # Create and start the pipeline thread properly.
        self.pipeline_thread = threading.Thread(target=self.run_pipeline, daemon=True)
        self.pipeline_thread.start()
        self.initialized = True

    def stop_pipeline(self):
        if self.pipeline:
            self.stop_event.set()
            self.initialized = False
            if self.pipeline_thread and self.pipeline_thread.is_alive():
                self.pipeline_thread.join()
                self.pipeline_thread = None
            self.pipeline.stop()
            self.pipeline = None

    def run_pipeline(self):
        self.pipeline.start()
        while not self.stop_event.is_set():
            # This iteration keeps the GLib main loop active for GStreamer callbacks.
            GLib.MainContext.default().iteration(False)

    def publish_frame(self):
        if not self.initialized or not self.pipeline:
            return
        frame, width, height = self.pipeline.get_frame()
        if frame is not None:
            msg = Image420()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            # Note: we now use JPEG encoding.
            msg.encoding = "jpeg"
            msg.width = width
            msg.height = height
            msg.data = frame
            self.image_pub.publish(msg)
            self.get_logger().info(f"Published frame with size: {len(msg.data)} bytes.")
        else:
            self.get_logger().warning("No frame available to publish.")

def main(args=None):
    rclpy.init(args=args)
    video_stream = VideoStreamNode()
    try:
        rclpy.spin(video_stream)
    except KeyboardInterrupt:
        pass
    finally:
        video_stream.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
