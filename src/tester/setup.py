from setuptools import find_packages, setup

package_name = 'tester'

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
            "imu_sim_node = tester.imu_sim_node:main",
            "mpc_sim_node = tester.mpc_sim_node:main",
            "data_logger = tester.data_logger:main",
            "cmd_vel_sim_node = tester.cmd_vel_sim_node:main",
            "canbus_tester_node = tester.canbus_test:main"
        ],
    },
)
