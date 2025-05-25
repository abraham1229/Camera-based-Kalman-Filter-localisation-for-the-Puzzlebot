import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from launch.actions import LogInfo, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LocalSubstitution


def generate_launch_description():
  
  # Parameters
  config = os.path.join(
      get_package_share_directory('final_challenge'),
      'config',
      'params.yaml'
      )
  
  path_generator_node = Node(
      package='final_challenge',
      executable='path_generator',
      output='screen',
      parameters=[config]
  )
    
  controller_node = Node(name="controller",
                          package='final_challenge',
                          executable='controller',
                          parameters=[config]
                          )
  
  odometry_node = Node(name="odometry",
                          package='final_challenge',
                          executable='odometry',
                          parameters=[config]
                          )


  l_d = LaunchDescription([
      controller_node,
      odometry_node,
      path_generator_node,
      ])

  return l_d