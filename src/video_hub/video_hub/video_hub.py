import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from gi.repository import Gst, GLib
import numpy as np
from video_hub.gs_pipeline import GStreamerPipeline


class VideoStreamNode(Node):
    def __init__(self):
        super().__init__('video_stream_publisher')
        self.pipeline = GStreamerPipeline("rtsp://192.168.126.200:554/av0_")
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'video_frames', 10)
        self.timer = self.create_timer(0.03, self.publish_frame)  # ~30 FPS
        self.pipeline.start()
        self.counter = 0

    def publish_frame(self):
        frame = self.pipeline.get_frame()
        if frame is not None:
            #cv2.imwrite(f"last_frame{self.counter}.jpg", frame)
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(ros_image)
            self.counter +=1
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