# final_challenge/launch/simulation_with_logic.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('puzzlebot_gazebo'),
                'launch',
                'bringup_simulation_simple_launch.py'
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
        gazebo,
        control
    ])
