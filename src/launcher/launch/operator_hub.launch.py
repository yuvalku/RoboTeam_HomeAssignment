import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    remote_controller = Node(
        package='operators_hub',
        name='ps_controller_node',
        executable='ps_controller_node',
        emulate_tty=True,
        output={ 
                'stdout': 'screen',
                'stderr': 'screen',
        }
    )
    
    status_disp = Node(
        package='operators_hub',
        name='rook_status_display_node',
        executable='rook_status_display_node',
        emulate_tty=True,
        output={ 
                'stdout': 'screen',
                'stderr': 'screen',
        }
    )

    
    ld.add_action(remote_controller)
    ld.add_action(status_disp)

    return ld