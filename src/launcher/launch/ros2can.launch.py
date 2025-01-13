import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    ros2can = Node(
        package='ros2can',
        name='ros2can_node',
        executable='ros2can_node',
        emulate_tty=True,
        output={ 
                'stdout': 'screen',
                'stderr': 'screen',
        }
    )
    
    status_node = Node(
        package='ros2can',
        name='can_ros2_status_node',
        executable='can_ros2_status_node',
        emulate_tty=True,
        output={ 
                'stdout': 'screen',
                'stderr': 'screen',
        }
    )

    
    ld.add_action(ros2can)
    ld.add_action(status_node)

    return ld