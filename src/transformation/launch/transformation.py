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
                "1.30317", "0.0174152", "0.675776",
                "-0.388123", "-0.0054127", "0.92155", "0.0087602",
                "base_link", "camera_link"
            ],
        ),
    ])