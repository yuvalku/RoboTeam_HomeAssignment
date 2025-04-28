from abc import ABC, abstractmethod

class RobotInterface(ABC):
    """
    Abstract base class for robot interfaces.
    """

    def __init__(self, id: int):
        self.id = id

    @abstractmethod
    def send_command(self, command: str):
        """
        Send a command to the robot.
        """
        pass

    @abstractmethod
    def receive_data(self) -> bytes:
        """
        Receive data from the robot.
        """
        pass

    @abstractmethod
    def send_status(self, status: str):
        """
        Send status information of the robot.
        """
        pass

    @abstractmethod
    def send_heartbeat(self):
        """
        Send a heartbeat signal to the robot.
        """
        pass
    