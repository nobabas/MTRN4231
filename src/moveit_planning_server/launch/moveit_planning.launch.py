from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='moveit_planning_server',
            executable='moveit_server_node',
            name='moveit_server_node',
            output='screen',
        ),
    ])