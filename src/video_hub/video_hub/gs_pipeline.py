# import gi
# gi.require_version('Gst', '1.0')
# from gi.repository import Gst
# import numpy as np

# Gst.init(None)

# class GStreamerPipeline:
#     def __init__(self, camera_url):
#         Gst.init(None)
#         print("here")
#         self.pipeline = None
#         self.appsink = None
#         self.create_pipeline(camera_url)

#     def create_pipeline(self, camera_url):
#         try:
#             self.pipeline = Gst.parse_launch(
#                 f"rtspsrc location={camera_url} latency=200 ! rtph264depay ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink name=sink emit-signals=true"
#             )
#             self.appsink = self.pipeline.get_by_name("sink")
#             if not self.appsink:
#                 raise RuntimeError("Failed to find appsink in the pipeline.")
#             self.appsink.set_property("max-buffers", 1)
#             self.appsink.set_property("drop", True)
#             self.appsink.set_property("emit-signals", True)
#         except Exception as e:
#             raise RuntimeError(f"Failed to create pipeline: {e}")

#     def start(self):
#         if self.pipeline:
#             self.pipeline.set_state(Gst.State.PLAYING)

#     def stop(self):
#         if self.pipeline:
#             self.pipeline.set_state(Gst.State.NULL)

#     def get_frame(self):
#         if not self.appsink:
#             return None

#         sample = self.appsink.emit("pull-sample")
#         if not sample:
#             return None

#         buf = sample.get_buffer()
#         caps = sample.get_caps()
#         width = caps.get_structure(0).get_value("width")
#         height = caps.get_structure(0).get_value("height")

#         success, map_info = buf.map(Gst.MapFlags.READ)
#         if not success:
#             return None

#         frame = np.frombuffer(map_info.data, dtype=np.uint8).reshape((height, width, 3))
#         buf.unmap(map_info)

#         return frame

from gi.repository import Gst
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import gi
import numpy as np
from threading import Thread
import time
import platform
gi.require_version('Gst', '1.0')

class CameraPublisher(Node):
    def __init__(self, node_name='video_cv2_hub_node'):
        super().__init__(node_name)
        self.get_logger().info('start cv_bridge_node')
        self.pipe = None
        self.declare_parameters(
            namespace='',
            parameters=[
                ('source', 'video0'),
                ('format', 'Tiltan'),
                ('input_fps', 30),
                ('output_fps', 30),
                ('h265', False),
                ('host', '10.0.0.19'),
                ('output_port', '9001'),
                ('input_port', '9001'),
                ('bitrate', 1500000),
                ('input_resolution', [1920, 1080]),
                ('output_resolution', [1280, 720]),
                ('recording', False),
            ])
        self.node_name = node_name
        self.input_resolution = [1920, 1080]
        self.output_resolution = [1280, 720]
        self.host = "rtsp://192.168.126.200:554/av0_"
            
        self.bitrate = 1500000

        self.input_width = self.input_resolution[0]
        self.input_height = self.input_resolution[1]

        self.pipe = None
        self.draw_plugin = None
        self.pre_in_track = False
        self.current_fov = 30
        self.debug = None
        self.zoom = None
        self.calculator_fps = 0.0
        self.calculator_delay = 0
        self.temp_calculator_all_delay = 0
        self.prev_frame_time = None
        self.frame_per_second_counter = 0
        self.frame_counter = 0
        self.recorder_bin = None
        self.image_row_publisher_ = self.create_publisher(
            Image, 'video_image_raw_data', 0)
        self.recorder_status = 0
        self.rtspsrc = "rtspsrc"
        self.nvv4l2enc = "nvv4l2h264enc"
        

    def init_gstreamer(self):
        Gst.init(None)
        
        if platform.uname().machine == 'x86_64':
            # Define the GStreamer pipeline
            self.pipe = Gst.parse_launch(
                'v4l2src device="/dev/video4" io-mode=2  '
                ' ! image/jpeg, width=3840, height=2160, framerate=30/1 ! '
                'video/x-raw, format=(string)UYVY ! '
                'appsink name=appsink emit-signals=true max-buffers=1 drop=true'
            )
        elif platform.uname().machine == 'aarch64':
            prefix_pipe = ""
            prefix_pipe = f'rtspsrc location={self.host}  \
                        caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, is-live=1, payload=(int)[96, 127], encoding-name=(string)H264" ! rtph264depay  ! queue ! h264parse  \
                         ! nvvidconv '
            # sss = (f' {prefix_pipe} '
            #        ' ! tee name=t t.  ! queue ! nvvidconv  '
            #        f' ! video/x-raw,width={self.output_width},height={self.output_height} '
            #        '  ! DrawPlugin name=draw_plugin '
            #        f' ! nvvidconv ! {self.nvv4l2enc} insert-sps-pps=1 idrinterval=30 insert-vui=1  maxperf-enable=1 iframeinterval=40 preset-level=1 control-rate=1 MeasureEncoderLatency=1   bitrate={self.bitrate} '
            #        f' ! {self.rtphpay} '
            #        ' ! queue   '
            #        f' ! udpsink host={self.host} port={self.output_port}  sync=false async=false'
            #        '  t. ! queue '
            #        ' ! nvvidconv ! video/x-raw, format=(string)I420  '
            #        ' ! appsink name=appsink emit-signals=true max-buffers=1 drop=true ')
            # print(sss)
            self.pipe = Gst.parse_launch(f' {prefix_pipe} '
                                         ' ! tee name=t t.  ! queue ! nvvidconv  '
                                         f' ! video/x-raw,width={self.input_width},height={self.input_height} '
                                         f' !  {self.nvv4l2enc} insert-sps-pps=1 idrinterval=30 insert-vui=1  maxperf-enable=1 iframeinterval=40 preset-level=1 control-rate=1 MeasureEncoderLatency=1   bitrate={self.bitrate} '
                                         f' ! {self.rtphpay} '
                                         ' ! queue   '
                                         f' ! udpsink host={self.host} port={self.output_port}  sync=false async=false'
                                         '  t. ! queue '
                                         ' ! nvvidconv ! video/x-raw, format=(string)I420  '
                                         ' ! appsink name=appsink emit-signals=true max-buffers=1 drop=true sync=false async=false ')

        self.appsink = self.pipe.get_by_name('appsink')
        self.appsink.connect('new-sample', self.on_new_sample)
        self.draw_plugin = self.pipe.get_by_name("draw_plugin")
        
        # Start the pipeline
        self.pipe.set_state(Gst.State.PLAYING)

    def calculator_per_sec(self, delay):
        if not self.prev_frame_time:
            self.prev_frame_time = time.time()
        current_time = time.time()
        if current_time - self.prev_frame_time < 1:
            self.frame_per_second_counter += 1
            self.temp_calculator_all_delay += delay
        else:
            self.calculator_fps = self.frame_per_second_counter
            self.frame_per_second_counter = 0
            self.prev_frame_time = current_time
            self.calculator_delay = 0 if self.calculator_fps == 0 else self.temp_calculator_all_delay // self.calculator_fps
            # self.calculator_delay += self.calculator_delay + 0
            self.temp_calculator_all_delay = 0

    def on_new_sample(self, appsink):
        sample = appsink.emit('pull-sample')
        if sample:
            buffer = sample.get_buffer()
            caps = sample.get_caps()
            height = caps.get_structure(0).get_value('height')
            width = caps.get_structure(0).get_value('width')
            inmap = buffer.map(Gst.MapFlags.READ)
            plane = np.frombuffer(inmap.data, dtype=np.uint8)[
                :].reshape((height*3//2, width)).flatten().tobytes()
            buffer.unmap(inmap)

            # Convert the image to a ROS2 message
            now = self.get_clock().now().to_msg()
            # ros_image = self.bridge.cv2_to_imgmsg(plane, encoding='yuv422')
            ros_image = Image()
            ros_image.header.stamp = now
            ros_image.data = plane
            ros_image.height = height
            ros_image.width = width
            ros_image.encoding = 'yuv420'
            self.get_logger().debug(
                 f'gst_image_frame_counter: {self.frame_counter}')

            # Publish the ROS2 message
            self.image_row_publisher_.publish(ros_image)

            

        return Gst.FlowReturn.OK

    

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    Thread(target=camera_publisher.init_gstreamer(), daemon=True).start()
    # tester
    # Thread(target=camera_publisher.change_delay_test(), daemon=True).start()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()