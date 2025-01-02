from setuptools import find_packages, setup

package_name = 'operators_hub'

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
            "ps_controller_node = operators_hub.ps_controller_node:main",
            "teleop_keyboard_node = operators_hub.teleop_keyboard_node:main",
            "rook_status_display_node = operators_hub.rook_status_display_node:main"
        ],
    },
)
