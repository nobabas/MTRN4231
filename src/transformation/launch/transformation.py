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
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_broadcaster',
            output='screen',
            arguments=[
                "1.27354", "0.0326318", "0.681138",
                "-0.397486", "-0.00834818", "0.91757", "-0.000592307",
                "base_link", "camera_link"
            ],
        ),
    ])