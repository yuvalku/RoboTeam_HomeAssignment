import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    drive_hub_node = Node(
        package='drive_hub',
        name='drive_hub_node',
        executable='drive_hub_node',
        output={'both': 'log'},
    )

    skid_steer = Node(
        package='drive_hub',
        name='skid_steer_node',
        executable='skid_steer_node',
        output={'both': 'log'},
    )
    
    ros2can = Node(
        package='ros2can',
        name='ros2can_node',
        executable='ros2can_node',
        output={'both': 'log'},
    )

    imu_tester = Node(
        package='tester',
        name='imu_sim_node',
        executable='imu_sim_node',
        output={ 
                'stdout': 'screen',
                'stderr': 'screen',
        }
    )

    mpc_tester = Node(
        package='tester',
        name='mpc_sim_node',
        executable='mpc_sim_node',
        emulate_tty=True,
        output={ 
                'stdout': 'screen',
                'stderr': 'screen',
        }
    )
    
    ld.add_action(drive_hub_node)
    ld.add_action(skid_steer)
    ld.add_action(ros2can)
    ld.add_action(imu_tester)
    ld.add_action(mpc_tester)

    return ld