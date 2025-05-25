from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    hackerboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('puzzlebot_ros'),
                'launch',
                'micro_ros_agent.launch.py'
            )
        )
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'view_rplidar_a1_launch.py'
            )
        )
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('puzzlebot_ros'),
                'launch',
                'camera_jetson.launch.py'
            )
        )
    )
    
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('final_challenge'),
                'launch',
                'control_launch.py'
            )
        )
    )

    return LaunchDescription([
        hackerboard,
        lidar,
        camera,
        control
    ])