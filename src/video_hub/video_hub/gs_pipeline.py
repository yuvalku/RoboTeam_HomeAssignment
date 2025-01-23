import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import numpy as np
import cv2

class GStreamerPipeline:
    def __init__(self, source):
        self.source = source
        self.running = False
        self.pipeline = None
        self.bus = None
        self.sample = None
        self.main_loop = None

        Gst.init(None)
        pipeline_str = (
            f"rtspsrc location={self.source} latency=60 ! "
                "rtph264depay ! h264parse ! nvv4l2decoder ! nvvidconv ! jpegenc ! "
                "queue max-size-buffers=5 leaky=downstream ! "
                "appsink name=appsink drop=true max-buffers=1"
        )

        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsink = self.pipeline.get_by_name("appsink")
        self.appsink.set_property("emit-signals", True)
        self.appsink.connect("new-sample", self.new_sample_callback)

    def new_sample_callback(self, sink):
        sample = sink.emit("pull-sample")
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
        if self.sample:
            buf = self.sample.get_buffer()
            data = buf.extract_dup(0, buf.get_size())
            frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
            return frame
        return None
    
    def save_frame(self, frame, filepath):
        """Save a single frame to a file."""
        cv2.imwrite(filepath, frame)
        print(f"Frame saved to {filepath}")
    

def main(args=None):
    pipeline = GStreamerPipeline("rtsp://192.168.126.200:554/av0_")
    pipeline.start()
    try:
        counter = 0
        print("Fetching a frame...")
        import time
        while True:
            time.sleep(0.1)  # Allow the pipeline to initialize and fetch some frames
            frame = pipeline.get_frame()

            if frame is not None:
                pipeline.save_frame(frame, f"output_frame{counter}.jpg")
                counter +=1
            else:
                print("No frame received.")
    finally:
        pipeline.stop()


if __name__=="__main__":
    main()
