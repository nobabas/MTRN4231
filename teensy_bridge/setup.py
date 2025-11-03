from setuptools import setup
from glob import glob

import os

package_name = 'teensy_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs', 'pyserial'],
    zip_safe=True,
    maintainer='sampcg',
    maintainer_email='sampcg@todo.todo',
    description='Bridge between Teensy sensor data and ROS 2 topics',
    license='MIT',
    entry_points={
        'console_scripts': [
            'teensy_bridge = teensy_bridge.teensy_bridge_node:main',
        ],
    },
)
