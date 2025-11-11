from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

import os

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='vision',
            executable='vision_main',
            name='vision_main',
            output='screen',
        ),
        Node(
            package='vision',
            executable='ros2node',
            name='ros2node',
            output='screen',
        ),
        Node(
            package='vision',
            executable='camera_run',
            name='camera_run',
            output='screen',
        ),
        
    ])