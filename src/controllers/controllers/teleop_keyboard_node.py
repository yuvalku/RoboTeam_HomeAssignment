import rclpy
from rclpy import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

def get_key(timeout=0.1):
    """
    Reads a single kestroke from stdin with given timeout (seconds)
    Returns the pressed key as a string or None if timeput occurs.
    """

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin],[], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = None
    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class KeyboardTeleop(Node):
    def __init__(self):    
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel_input', 10)
        self.settins = termios.tcgetattr(sys.stdin)
        self.speed_modes = {
            '1' : 10.0,   #slow
            '2' : 20.0,   #medium
            '3' : 30.0    #fast
        }

        self.current_speed = 20.0

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.timer = self.create_timer(0.1, self.publisher_cmd)

        self.get_logger().info("Keyboard teleop started.")
        self.get_logger().info("Use arrow keys to move (↑ ↓ ← →).")
        self.get_logger().info("Press 1,2,3 to change speed modes. Press q to quit.")

    def publish_cmd(self):
        """
        Publish the current Twist command at a regular interval.
        The linear_vel/angular_vel are updated in response to keypresses.
        """
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.angular.z = self.angular_velocity

        self.publisher_.publish(twist)
    
    def spin_teleop(self):
        """
        Main loop that continuously checks for keypresses and updates velocities.
        """
        try:
            while rclpy.ok():
                key = get_key(timeout=0.1)
                if key is None:
                    continue

                if key =='\x03' or key == 'q':
                    break

                if key in self.speed_modes:
                    self.current_speed = self.speed_modes[key]
                    self.get_logger().info(f"Speed set to {self.current_speed}")

                elif key == '\x1b':
                    key2 = get_key(timeout=0.1)
                    key3 = get_key(timeout=0.1)

                    if key2 == '[':
                        if key3 == 'A': # UP arrow
                            self.linear_velocity = self.current_speed
                            self.angular_velocity = 0.0
                        elif key3 == 'B': # Down arrow
                            self.linear_velocity = -self.current_speed
                            self.angular_velocity = 0.0
                        elif key3 == 'C': # Right arrow
                            self.linear_velocity = 0.0
                            self.angular_velocity = -self.current_speed
                        elif key3 == 'D': # Left arrow
                            self.linear_velocity = 0.0
                            self.angular_velocity = self.current_speed    

                else:
                    self.linear_velocity = 0.0
                    self.angular_velocity = 0.0

        except Exception as e:
            self.get_logger().error(str(e))
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settins)

            self.stop()

    def stop(self):
        self.get_logger().info("Stopping the robot")
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.publish_cmd()


def main(args=None):
    rclpy.init(args=args)
    teleop_node = KeyboardTeleop()
    try:
        teleop_node.spin_teleop()
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == "_main__":
    settings = termios.tcgetattr(sys.stdin)
    main()