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

    path_generator_node = Node(
        name='path_generator',
        package='final_challenge',
        executable='path_generator',
        parameters=[config]
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

<<<<<<< HEAD
=======
    aruco_node = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        parameters=[
                {"cam_base_topic": "/image_raw"},
                {"marker_size": 0.045},
                {"marker_dict": "4X4_50"},
                {"publish_tf": True}
        ],
        remappings=[],
        output='screen'
    )

    rqt_image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        output='screen'
    )

>>>>>>> real_robot_test
    return LaunchDescription([
        path_generator_node,
        controller_node,
        odometry_node,
    ])
