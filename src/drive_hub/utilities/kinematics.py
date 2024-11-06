import xml.etree.ElementTree as ET
import os
import numpy as np

class DifferentialDrive:
    def __init__(self, motor_radius, motor_separation, num_motors):
        self.motor_radius = motor_radius
        self.motor_separation = motor_separation
        self.num_motors = num_motors

    def calculate_motors_velocities(self, linear_velocity, angular_velocity):
        if self.num_motors == 4:
            left_motor_velocity = (linear_velocity - (angular_velocity * self.motor_separation / 2)) / self.motor_radius
            right_motor_velocity = (linear_velocity + (angular_velocity * self.motor_separation / 2)) / self.motor_radius
            return left_motor_velocity, right_motor_velocity
        elif self.num_motors == 6:
            left_motor_velocity = (linear_velocity - (angular_velocity * self.motor_separation / 3)) / self.motor_radius
            right_motor_velocity = (linear_velocity + (angular_velocity * self.motor_separation / 3)) / self.motor_radius
            return left_motor_velocity, right_motor_velocity
        else:
            raise ValueError("Unsupported number of motors: {}".format(self.num_motors))

def load_settings(filename):
    current_dir = os.path.dirname(os.path.realpath(__file__))
    project_root = os.path.abspath(os.path.join(current_dir, ".."))
    settings_path = os.path.join(project_root, 'utilities', filename)

    tree = ET.parse(settings_path)
    root = tree.getroot()

    num_motors = int(root.find('num_motors').text)
    motor_radius = float(root.find('motor_radius').text)
    motor_separation = float(root.find('motor_separation').text)

    return num_motors, motor_radius, motor_separation        

def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw.
        quaternion = [x, y, z, w]
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw