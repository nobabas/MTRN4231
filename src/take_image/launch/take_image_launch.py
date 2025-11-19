from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

import os

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='take_image',
            executable='camera_run',
            name='camera_run',
            output='screen',
        ),
        Node(
            package='take_image',
            executable='ros2node',
            name='ros2node',
            output='screen',
        ),
    ])