import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from gi.repository import Gst, GLib
import numpy as np
from video_hub.gs_pipeline import GStreamerPipeline


class VideoStreamNode(Node):
    def __init__(self):
        super().__init__('video_stream_publisher')
        self.pipeline = None
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'video_frames', 10)
        self.camera_ip_sub = self.create_subscription(String, 'camera_ip', self.update_camera_ip, 10)
        self.timer = self.create_timer(0.03, self.publish_frame)  # ~30 FPS
        self.current_ip = None
        self.initialized = False
        self.counter = 0

    def update_camera_ip(self, msg):
        new_ip = msg.data
        if new_ip != self.current_ip:
            self.current_ip = new_ip
            self.restart_pipeline(new_ip)
            self.get_logger().info(f"Updated pipeline to new IP: {new_ip}")

    def restart_pipeline(self, ip):
        if self.pipeline:
            self.pipeline.stop()
        self.pipeline = GStreamerPipeline(ip)
        self.pipeline.start()
        self.initialized = True

    def publish_frame(self):
        if not self.initialized or not self.pipeline:
            return

        frame = self.pipeline.get_frame()
        if frame is not None:
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(ros_image)
        else:
            self.get_logger().warning("Failed to get frame from pipeline.")

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


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