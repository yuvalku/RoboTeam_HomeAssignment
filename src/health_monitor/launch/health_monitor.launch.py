import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    health_analyzer = Node(
            package='health_monitor',
            executable='health_analyzer_node',
            name='health_analyzer',
            output='screen',
            emulate_tty=True
        )

    alert_manager = Node(
            package='health_monitor',
            executable='alert_manager',
            name='alert_manager',
            output='screen',
            emulate_tty=True
        )
    
    can_ros2_status = Node(
            package='ros2can', 
            executable='can_ros2_status_node',
            name='can_status_node',
            output='screen',
            emulate_tty=True 
        )

    return LaunchDescription([
        health_analyzer,
        alert_manager,
        can_ros2_status
    ])
