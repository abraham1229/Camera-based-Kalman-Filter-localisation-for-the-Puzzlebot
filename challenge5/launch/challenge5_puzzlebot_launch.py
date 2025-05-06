import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import  EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LocalSubstitution


def generate_launch_description():

  # Parameters
  config = os.path.join(
      get_package_share_directory('challenge5'),
      'config',
      'params.yaml'
      )
  
  path_generator_node = Node(
      package='challenge5',
      executable='path_generator',
      output='screen',
      parameters = [config]
  )
    
  controller_node = Node(name="controller",
                          package='challenge5',
                          executable='controller'
                          )
  
  odometry_node = Node(name="odometry",
                          package='challenge5',
                          executable='odometry'
                          )


  # Ensure full shutdown when SIGINT (Ctrl+C) is received
  shutdown_log = RegisterEventHandler(
                                  OnShutdown(
                                      on_shutdown=[LogInfo(
                                          msg=['Launch was asked to shutdown: ',
                                              LocalSubstitution('event.reason')]
                                      )]
                                  )
                              )

  l_d = LaunchDescription([
      controller_node,
      odometry_node,
      shutdown_log,
      path_generator_node
      ])

  return l_d