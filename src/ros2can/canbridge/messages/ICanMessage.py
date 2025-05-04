from abc import ABC, abstractmethod
#from can_parser import parse_frame
class CanMessage(ABC):
    """
    Abstract base class for CAN messages.
    """

    def __init__(self, id: int, data: bytes = None):
        self.id = id
        self.data = data

    @abstractmethod
    def set_velocity_message(self, left, right) -> bytes:
        """
        Convert the velocity message to bytes.
        """
        pass
    
    @abstractmethod
    def set_joint_speed_message(self, joint_id: int, speed: int) -> bytes:
        """
        Convert the joint speed message to bytes.
        """
        pass
    
    # @abstractmethod
    # def set_heartbeat_message(self) -> bytes:
    #     """
    #     Convert the heartbeat message to bytes.
    #     """
    #     pass

    # @abstractmethod
    # def set_ir_led_message(self, set_led: bool) -> bytes:
    #     """
    #     Convert the IR LED message to bytes.
    #     """
    #     pass

    # @abstractmethod
    # def set_white_led_message(self, set_led: bool) -> bytes:
    #     """
    #     Convert the white LED message to bytes.
    #     """
    #     pass

    # @abstractmethod
    # def get_data(self, data: bytes):
    #     """
    #     Convert bytes to the message.
    #     """
    #     pass

    # @abstractmethod
    # def parse_frame(self, frame: bytes):
    #     """
    #     Parse a CAN frame.
    #     """
    #     pass

    @staticmethod
    def get_message(bus, timeout=1.0):
        """
        Receive a CAN message from the provided CAN bus.
        """
        msg = bus.recv(timeout=timeout)
        if msg:
            print(f"Message received: ID={msg.arbitration_id}, Data={msg.data}")
            return msg.arbitration_id, msg.data
        else:
            print("No message received.")
            return None, None