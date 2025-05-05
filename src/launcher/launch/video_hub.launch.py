import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    video_hub = Node(
        package='video_hub',
        name='video_hub',
        executable='video_hub',
        emulate_tty=True,
        output={
                'stdout': 'screen',
                'stderr': 'screen',
        }
    )

    ld.add_action(video_hub)
    return ld