import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

# Toggle between simulated or real UR5e hardware
use_fake = True
use_fake_str = 'true'
ur_type = 'ur5e'
ip_address = 'yyy.yyy.yyy.yyy'

if not use_fake:
    ip_address = '192.168.0.100' # Change this to robot's IP
    use_fake_str = 'false'


def get_ur_control_launch():
    """Configure UR control launch for the UR5e arm."""

    end_effector_path = os.path.join(
        get_package_share_directory('endeffector_description'), 'urdf', 'ur_with_endeffector.xacro'
    )

    ur_control_launch_args = {
        'ur_type': ur_type,
        'robot_ip': ip_address,
        'use_fake_hardware': use_fake_str,
        'launch_rviz': 'false', # We set this to false because MoveIt will launch RViz
        'description_file': end_effector_path, 
    }

    # Add controller if using simulated hardware
    if use_fake:
        ur_control_launch_args['initial_joint_controller'] = 'scaled_joint_trajectory_controller'

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ur_robot_driver'), 'launch', 'ur_control.launch.py'])
        ),
        launch_arguments=ur_control_launch_args.items(),
    )


def get_moveit_launch():
    """Configure MoveIt launch with a delay to ensure UR control is initialized."""
    moveit_launch_args = {
        'ur_type': ur_type,
        'launch_rviz': 'true', 
        'use_fake_hardware': use_fake_str,
    }

    return TimerAction(
        period=10.0, # Delay to prevent conflicts
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('ur_moveit_config'), 'launch', 'ur_moveit.launch.py'])
                ),
                launch_arguments=moveit_launch_args.items(),
            )
        ]
    )


def generate_launch_description():
    """Main function to generate the complete launch description."""
    launch_description = [
        get_ur_control_launch(),
        get_moveit_launch(),
    ]
    return LaunchDescription(launch_description)
