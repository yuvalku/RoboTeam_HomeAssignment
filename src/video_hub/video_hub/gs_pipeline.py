# gs_pipeline.py
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import numpy as np
import cv2

class GStreamerPipeline:
    def __init__(self, source):
        self.source = source
        self.running = False
        self.pipeline = None
        self.sample = None

        Gst.init(None)
        # Updated pipeline:
        #  - rtspsrc grabs the stream from the RTSP source
        #  - rtph264depay and h264parse extract the H.264 stream
        #  - avdec_h264 decodes it
        #  - videoconvert makes sure the pixel format is compatible with jpegenc
        #  - jpegenc (with quality=70) re‑encodes as JPEG
        #  - appsink provides the frames to our callback.
        pipeline_str = (
            f"rtspsrc location={self.source} latency=80 ! "
            "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
            "videoscale ! video/x-raw,width=720,height=480 ! "
            "jpegenc quality=45 ! appsink name=appsink emit-signals=true max-buffers=1 drop=true"
        )

        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsink = self.pipeline.get_by_name("appsink")
        self.appsink.connect("new-sample", self.new_sample_callback)

    def new_sample_callback(self, appsink):
        # Pull the new sample from appsink and store it for retrieval.
        sample = appsink.emit("pull-sample")
        self.sample = sample
        return Gst.FlowReturn.OK

    def start(self):
        if not self.running:
            self.running = True
            self.pipeline.set_state(Gst.State.PLAYING)

    def stop(self):
        if self.running:
            self.pipeline.set_state(Gst.State.NULL)
            self.running = False

    def get_frame(self):
        if not self.sample:
            # No sample available yet
            return None, None, None

        buf = self.sample.get_buffer()
        success, mapinfo = buf.map(Gst.MapFlags.READ)
        if not success:
            print("Failed to map buffer.")
            return None, None, None

        # Get the JPEG frame as bytes
        jpeg_frame = mapinfo.data
        caps = self.sample.get_caps()
        structure = caps.get_structure(0)
        try:
            width = structure.get_value('width')
            height = structure.get_value('height')
        except Exception as e:
            # If width/height aren’t available in the caps, decode the JPEG to find out.
            np_frame = np.frombuffer(jpeg_frame, dtype=np.uint8)
            frame_decoded = cv2.imdecode(np_frame, cv2.IMREAD_COLOR)
            if frame_decoded is not None:
                height, width, _ = frame_decoded.shape
            else:
                width, height = 0, 0

        buf.unmap(mapinfo)
        return jpeg_frame, width, height

    def save_frame(self, frame, filepath):
        """Save a single frame to a file."""
        with open(filepath, 'wb') as f:
            f.write(frame)
        print(f"Frame saved to {filepath}")

# You can test this module independently if needed.
if __name__=="__main__":
    pipeline = GStreamerPipeline("rtsp://192.168.126.200:554/av0_")
    pipeline.start()
    try:
        counter = 0
        import time
        while True:
            time.sleep(0.1)  # Give the pipeline time to start
            frame, width, height = pipeline.get_frame()
            if frame is not None:
                pipeline.save_frame(frame, f"output_frame{counter}.jpg")
                counter += 1
            else:
                print("No frame received.")
    finally:
        pipeline.stop()
