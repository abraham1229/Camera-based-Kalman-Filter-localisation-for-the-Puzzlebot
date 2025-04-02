from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Define URDF file path
    urdf_file_path = '/home/ai31/ros2_ws/src/files/challenge1/urdf/puzzle.urdf'
    # Define paths
    package_name = 'challenge1'
    urdf_file_name = 'puzzle.urdf'
    package_share_path = FindPackageShare(package_name)
    urdf_default_path = PathJoinSubstitution([package_share_path, 'urdf', urdf_file_name])

    # Declare launch arguments
    gui_arg = DeclareLaunchArgument(
        name='gui', default_value='true', choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui')
    
    model_arg = DeclareLaunchArgument(
        name='model', default_value=urdf_default_path,
        description='Path to robot URDF file')
    
    # Read the URDF file
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()
    
    # Define nodes
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )
    
    # Include additional launch file (if needed, modify the package and path)
    display_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': package_name,
            'urdf_package_path': LaunchConfiguration('model'),
            'jsp_gui': LaunchConfiguration('gui')
        }.items()
    )

    # Create launch description
    ld = LaunchDescription([
        gui_arg,
        model_arg,
        robot_state_pub_node,
        display_launch
    ])

    return ld
