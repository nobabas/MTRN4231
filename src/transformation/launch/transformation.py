from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

import os

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='transformation',
            executable='main',
            name='tf_main',
            output='screen',
        ),
    ])