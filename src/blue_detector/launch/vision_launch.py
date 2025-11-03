from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

import os

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='blue_detector',
            executable='PublishFinal',
            name='PublishFinal',
            output='screen',
        ),
        Node(
            package='blue_detector',
            executable='ros2node',
            name='ros2node',
            output='screen',
        )
    ])