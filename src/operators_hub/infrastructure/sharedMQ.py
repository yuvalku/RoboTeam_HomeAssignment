import zmq
from threading import Thread, Lock
import queue
import numpy as np
import cv2

class ZMQModule:
    def __init__(self, host='127.0.0.1', port_in=5557, video_port_out=5556):
        self.host = host
        self.port_in = port_in
        self.video_port_out = video_port_out
        self.context = zmq.Context()

        # PUB socket for sending data
        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.bind(f"tcp://{self.host}:{self.video_port_out}")  # Bind to all available interfaces for sending

        # SUB socket for receiving data
        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.bind(f"tcp://{self.host}:{self.port_in}")
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all topics

        # Thread-safe message queue for received messages
        self.message_queue = queue.Queue()
        self.running = True
        self.lock = Lock()

        # Start receiving thread
        self.receiver_thread = Thread(target=self._receive_messages, daemon=True)
        self.receiver_thread.start()

    def _receive_messages(self):
        while self.running:
            try:
                message = self.sub_socket.recv_string(flags=zmq.NOBLOCK)
                with self.lock:
                    self.message_queue.put(message)
            except zmq.Again:
                pass  # No messages to receive

    def get_message(self):
        with self.lock:
            try:
                topic, data = self.message_queue.get_nowait()
                return Message(topic=topic, data=data)
            except queue.Empty:
                return None

    def send_message(self, topic, message):
        try:
            # Always send as multipart
            if isinstance(message, bytes):
                self.pub_socket.send_multipart([topic.encode(), message])
            elif isinstance(message, np.ndarray):
                _, encoded_image = cv2.imencode('.jpg', message)  # Encode NumPy array as JPEG
                self.pub_socket.send_multipart([topic.encode(), encoded_image.tobytes()])
            else:
                self.pub_socket.send_multipart([topic.encode(), message.encode()])
        except Exception as e:
            print(f"Error sending message: {e}")

    def close(self):
        self.running = False
        self.receiver_thread.join()  # Wait for the receiver thread to finish
        self.pub_socket.close()
        self.sub_socket.close()
        self.context.term()

class Message:
    def __init__(self, topic, data):
        self.topic = topic
        self.data = data

    def __repr__(self):
        return f"Message(topic='{self.topic}', data={self.data})"