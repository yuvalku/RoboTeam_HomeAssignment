from setuptools import find_packages, setup

package_name = 'drive_hub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "drive_hub_node = drive_hub.drive_hub_node:main",
            "imu_sim_node = drive_hub.imu_sim_node:main",
            "mpc_sim_node = drive_hub.mpc_sim_node:main",
            "data_logger = drive_hub.data_logger:main",
            "cmd_vel_sim_node = drive_hub.cmd_vel_sim_node:main"
        ],
    },
)
