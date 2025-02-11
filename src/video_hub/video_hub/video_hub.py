"""
Video Hub Node for ROS 2
This node fetches a video stream from a camera using GStreamer, processes it, and publishes frames
to a ROS 2 topic (`video_frames`) using a custom message format (`Image420`).

It also listens for camera IP updates and dynamically restarts the GStreamer pipeline accordingly.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from interfaces.msg import Image420
from std_msgs.msg import String, Header
from cv_bridge import CvBridge
from gi.repository import GLib
import threading
from video_hub.gs_pipeline import GStreamerPipeline
from interfaces.srv import GetCameraIp
from time import sleep


class VideoHub(Node):
    """
    A ROS 2 node that manages a GStreamer video stream and publishes frames as `Image420` messages.
    It also handles dynamic updates to the camera's IP address.
    """

    def __init__(self):
        super().__init__('video_stream_publisher')

        self.pipeline = None
        self.pipeline_thread = None
        self.bridge = CvBridge()
        self.current_ip = None
        self.initialized = False
        self.stop_event = threading.Event()

        qos_video_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE, 
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        # Publishers and Subscribers
        self.image_pub = self.create_publisher(Image420, 'video_frames', qos_video_profile)
        self.camera_ip_sub = self.create_subscription(String, 'camera_ip_change', self.update_camera_ip, 10) 

        # Service client for fetching the initial camera IP
        self.ip_client = self.create_client(GetCameraIp, 'get_camera_ip')

        # Timer for publishing vdeo frames at 15 FPS
        self.timer = self.create_timer(1.0 / 15.0, self.publish_frame)
        
        # Request for initial IP address to set the pipeline
        self.request_initial_ip()

    def request_initial_ip(self) -> None:
        """
        Requests the initial camera IP from the `get_camera_ip` service and initializes the GStreamer pipeline.
        """        
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
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def update_camera_ip(self, msg) -> None:
        """
        Callback for handling camera IP updates. Restarts the GStreamer pipeline if the IP has changed.
        """
        new_ip = msg.data
        if new_ip != self.current_ip:
            self.current_ip = new_ip
            self.restart_pipeline(new_ip)
            self.get_logger().info(f"Updated pipeline to new IP: {new_ip}")

    def restart_pipeline(self, ip) -> None:
        """
        Stops the current pipeline (if any) and starts a new GStreamer pipeline using the specified IP.
        """
        self.stop_pipeline()
        self.pipeline = GStreamerPipeline(ip)
        self.stop_event.clear()
        self.pipeline_thread = threading.Thread(target=self.run_pipeline, daemon=True).start()
        self.initialized = True

    def stop_pipeline(self) -> None:
        """
        Gracefully stops the current GStreamer pipeline and its associated thread.
        """
        if self.pipeline:
            self.stop_event.set()
            self.initialized = False

            if self.pipeline_thread and self.pipeline_thread.is_alive():
                self.pipeline_thread.join()
                self.pipeline_thread = None

            self.pipeline.stop()
            self.pipeline = None

    def run_pipeline(self) -> None:
        """
        Runs the GStreamer pipeline and processes its main loop.
        This keeps the GLib context active for handling callbacks.
        """
        self.pipeline.start()

        while not self.stop_event.is_set():
            # This iteration keeps the GLib main loop active for GStreamer callbacks.
            GLib.MainContext.default().iteration(False)

    def publish_frame(self) -> None:
        """
        Publishes a video frame to the `video_frames` topic.
        Each frame is converted to the `Image420` message format.
        frames can be manipulated for image processing at this point using 
        get_frame() method before publishing it.
        """
        if not self.initialized or not self.pipeline:
            return
        frame, width, height = self.pipeline.get_frame()
        
        if frame is not None:
            msg = Image420()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.encoding = "jpeg"
            msg.width = width
            msg.height = height
            msg.data = frame
            if self.image_pub.get_subscription_count() > 0:
                self.image_pub.publish(msg)
                self.get_logger().info(f"Published frame with size: {len(msg.data)} bytes.", throttle_duration_sec=1)
            else:
                self.get_logger().info(f"no subscribers available")
                sleep(1)
        else:
            self.get_logger().warning("No frame available to publish.", throttle_duration_sec=1)

def main(args=None):
    rclpy.init(args=args)
    video_stream = VideoHub()
    try:
        rclpy.spin(video_stream)
    except KeyboardInterrupt:
        pass
    finally:
        video_stream.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
