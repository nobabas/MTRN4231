import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

# Need to first launch the arduino file before this launch file

def generate_launch_description():

    os.environ["XDG_SESSION_TYPE"] = "xcb"
    
    
    brain_node = Node(
            package='brain',
            executable='brain',  # Replace with the actual executable name
            name='brain_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    
    robotAndCamera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('endeffector_description'), 'launch'),
                '/ur_with_endeffector.launch.py'])
            )
    
    vision = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('vision'), 'launch'),
                '/vision_launch.py'])
            )
    
    transformations = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('transformation'), 'launch'),
                '/transformation.py'])
            )

    # Should add the movement here 
    # Question: is there a launch file for the moveit server?
    arm = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('moveit_planning_server'), 'launch'),
                '/moveit_planning.launch.py'])
            )
    
    # Teensy bridge launch should be after the arduino launch
    teensy_bridge = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('teensy_bridge'), 'launch'),
                '/teensy_bridge_launch.py'])
            )
    return LaunchDescription([
        # Declare launch arguments (optional)
        DeclareLaunchArgument('use_sim_time', default_value='false', 
                               description='Use simulation time if true'),

        brain_node,
        robotAndCamera,
        vision,
        arm,
        teensy_bridge,
        transformations,
    ])