# ps_joystick_controller.py
import rclpy
import pygame
from .ros2operator_interface import ROS2OperatorInterface

class PSJoystickController:
    def __init__(self):
        self.ros2interface = ROS2OperatorInterface()
        self.timer = self.ros2interface.create_timer(0.1, self.on_joystick_controller)

        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        self.display_video = False
        self.e_stop_active = False

        self.joint_names = ["pan", "shoulder", "elbow1", "elbow2", "wrist", "gripper"]
        self.current_joint_index = 0
        self.manipulator_mode = False
        self.last_hat_x = 0
        self.last_lb_state = False
        
        self.speed_levels = ["low", "medium", "fast"]
        self.current_speed_level = 0 # 0 = low, 1 = medium, 2 = fast ; default is low  
        self.last_rb_state = False
        
        self.joint_max_speeds = {
            "pan": 1000,
            "shoulder": 1000,
            "elbow1": 1500,
            "elbow2": 150,
            "wrist": 150,
            "gripper": 500,  # TODO: implement gripper control
        }

    
    def on_joystick_controller(self):
        pygame.event.pump()

        if self.joystick.get_button(1):
            self.e_stop_active = not self.e_stop_active
            self.ros2interface.get_logger().info(f"E-stop {'activated' if self.e_stop_active else 'deactivated'}.")
            pygame.time.wait(300)
        if self.e_stop_active:
            self.ros2interface.publish_velocity(0.0, 0.0)
            return
        
        if self.joystick.get_button(0):
            self.display_video = not self.display_video
            self.ros2interface.get_logger().info(f"Video display {'enabled' if self.display_video else 'disabled'}.")
            if self.display_video:
                self.ros2interface.start_video_subscription()
            else:
                self.ros2interface.stop_video_subscription()
            pygame.time.wait(300)
        
        if self.joystick.get_button(4) and self.display_video:
            self.ros2interface.get_logger().info("Camera toggling mode activated.")
            new_ip = self.ros2interface.camera_manager.toggle_camera()
            self.ros2interface.publish_camera_ip(new_ip)
            self.ros2interface.restart_video_subscription()
            pygame.time.wait(300)

        lb_state = self.joystick.get_button(6)
        rb_state = self.joystick.get_button(7)

        if lb_state and not self.last_lb_state:
            self.manipulator_mode = not self.manipulator_mode
            self.ros2interface.get_logger().info(f"Manipulator mode {'enabled' if self.manipulator_mode else 'disabled'}.")
            pygame.time.wait(300)
        self.last_lb_state = lb_state

        if rb_state and not self.last_rb_state:
            self.current_speed_level = (self.current_speed_level + 1) % len(self.speed_levels)
            self.ros2interface.get_logger().info(f"Speed mode changed to: {self.speed_levels[self.current_speed_level]}")
            pygame.time.wait(300)
        self.last_rb_state = rb_state
    
        if self.manipulator_mode:
            hat_x, hat_y = self.joystick.get_hat(0)
            if hat_x != self.last_hat_x:
                if hat_x == 1:
                    self.current_joint_index = (self.current_joint_index + 1) % len(self.joint_names)
                    self.ros2interface.get_logger().info(f"Joint selected: {self.joint_names[self.current_joint_index]}")
                    pygame.time.wait(200)
                elif hat_x == -1:
                    self.current_joint_index = (self.current_joint_index - 1) % len(self.joint_names)
                    self.ros2interface.get_logger().info(f"Joint selected: {self.joint_names[self.current_joint_index]}")
                    pygame.time.wait(200)
            self.last_hat_x = hat_x
            
            base_speed = 0
            if hat_y == 1:  # UP
                base_speed = 1
            elif hat_y == -1:  # DOWN
                base_speed = -1
            else:
                base_speed = 0
            joint_name = self.joint_names[self.current_joint_index]
            max_speed = self.joint_max_speeds.get(joint_name, 500)  # Default fallback
            speed_factor = [0.2, 0.5, 1.0][self.current_speed_level]
            joint_speed = base_speed * int(max_speed * speed_factor)

            self.ros2interface.publish_manipulator_control(joint_name, speed=joint_speed)

            if base_speed != 0:  
                self.ros2interface.get_logger().info(f"Manipulator control: {joint_name} moving at {joint_speed} (speed mode {self.speed_levels[self.current_speed_level]})")

        linear_velocity = -self.joystick.get_axis(1) * 500.0
        angular_velocity = self.joystick.get_axis(0) * 500.0
        if abs(linear_velocity) < 30.0 and abs(angular_velocity) < 30.0:
            linear_velocity = 0.0
            angular_velocity = 0.0
        self.ros2interface.publish_velocity(linear_x=linear_velocity, angular_z=angular_velocity)
        

def main(args=None):
    rclpy.init(args=args)
    joystick_controller = PSJoystickController()
    rclpy.spin(joystick_controller.ros2interface)
    joystick_controller.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__ == "__main__":
    main()
