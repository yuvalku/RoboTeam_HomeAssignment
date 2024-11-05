import xml.etree.ElementTree as ET
import os
import numpy as np

class DifferentailDrive:
    def __init__(self, wheel_radius, wheel_separation, num_wheels):
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation
        self.num_wheels = num_wheels

    def calculate_wheels_velocities(self, linear_velocity, angular_velocity):
        if self.num_wheels == 4:
            left_wheel_velocity = (linear_velocity - (angular_velocity * self.wheel_separation / 2)) / self.wheel_radius
            right_wheel_velocity = (linear_velocity + (angular_velocity * self.wheel_separation / 2)) / self.wheel_radius
            return left_wheel_velocity, right_wheel_velocity
        elif self.num_wheels == 6:
            left_wheel_velocity = (linear_velocity - (angular_velocity * self.wheel_separation / 3)) / self.wheel_radius
            right_wheel_velocity = (linear_velocity + (angular_velocity * self.wheel_separation / 3)) / self.wheel_radius
            return left_wheel_velocity, right_wheel_velocity
        else:
            raise ValueError("Unsupported number of wheels: {}".format(self.num_wheels))

def load_settings(filename):
    current_dir = os.path.dirname(os.path.realpath(__file__))
    project_root = os.path.abspath(os.path.join(current_dir, ".."))
    settings_path = os.path.join(project_root, 'utilities', filename)

    tree = ET.parse(settings_path)
    root = tree.getroot()

    num_wheels = int(root.find('num_wheels').text)
    wheel_radius = float(root.find('wheel_radius').text)
    wheel_separation = float(root.find('wheel_separation').text)

    return num_wheels, wheel_radius, wheel_separation        

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