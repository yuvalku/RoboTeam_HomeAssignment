import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    health_analyzer = Node(
        package='health_monitor',
        executable='health_analyzer_node',
        name='health_analyzer',
        emulate_tty=True,
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        }
    )

    alert_manager = Node(
        package='health_monitor',
        executable='alert_manager_node',
        name='alert_manager',
        emulate_tty=True,
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        }
    )

    can_ros2_status = Node(
        package='ros2can',
        executable='can_ros2_status_node',
        name='can_status_provider',
        emulate_tty=True,
        output={
            'stdout': 'screen',
            'stderr': 'screen',
    )
        

    return LaunchDescription([
        can_ros2_status,
        health_analyzer,
        alert_manager
    ])
