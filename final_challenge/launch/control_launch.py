# final_challenge/launch/control_logic.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('final_challenge'),
        'config',
        'params.yaml'
    )

    controller_node = Node(
        name='controller',
        package='final_challenge',
        executable='controller',
        parameters=[config]
    )

    odometry_node = Node(
        name='odometry',
        package='final_challenge',
        executable='odometry',
        parameters=[config]
    )

    path_generator_node = Node(
        name='path_generator',
        package='final_challenge',
        executable='path_generator',
        parameters=[config]
    )

    return LaunchDescription([
        controller_node,
        odometry_node,
        path_generator_node
    ])
