import zmq
from threading import Thread, Lock
import queue
import numpy as np
import cv2

class ZMQModule:
    def __init__(self, host='10.0.70.24', port_in=5555, port_out=5556):
        """
        Initialize the ZMQ module with separate channels for receiving and sending.
        
        Args:
            host (str): IP address of the server.
            port_in (int): Port for incoming messages (SUB socket).
            port_out (int): Port for outgoing messages (PUB socket).
        """
        self.host = host
        self.port_in = port_in
        self.port_out = port_out
        self.context = zmq.Context()

        # PUB socket for sending data
        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.bind(f"tcp://0.0.0.0:{self.port_out}")  # Bind to all available interfaces for sending

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
        """
        Continuously receive messages from the SUB socket and add them to the queue.
        """
        while self.running:
            try:
                message = self.sub_socket.recv_string(flags=zmq.NOBLOCK)
                with self.lock:
                    self.message_queue.put(message)
            except zmq.Again:
                pass  # No messages to receive

    def get_message(self):
        """
        Retrieve the latest message from the queue in a thread-safe manner.
        
        Returns:
            str: The next message from the queue, or None if the queue is empty.
        """
        with self.lock:
            try:
                return self.message_queue.get_nowait()
            except queue.Empty:
                return None

    def send_message(self, topic, message):
        """
        Send a message through the PUB socket.
        
        Args:
            topic (str): The topic of the message.
            message (str | np.ndarray): The message content. If it's a NumPy array, it will be sent as an image.
        """
        try:
            if isinstance(message, np.ndarray):
                _, encoded_image = cv2.imencode('.jpg', message)  # Encode NumPy array as JPEG
                self.pub_socket.send_multipart([topic.encode(), encoded_image.tobytes()])
            else:
                self.pub_socket.send_string(f"{topic} {message}")
        except Exception as e:
            print(f"Error sending message: {e}")

    def close(self):
        """
        Cleanly close the ZMQ sockets and terminate the context.
        """
        self.running = False
        self.receiver_thread.join()  # Wait for the receiver thread to finish
        self.pub_socket.close()
        self.sub_socket.close()
        self.context.term()

