from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teensy_bridge',
            executable='teensy_bridge',
            name='teensy_bridge',
            output='screen'
        ),
    ])
