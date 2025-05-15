import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import LogInfo, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LocalSubstitution


def generate_launch_description():

  init_x = 0.0
  init_y = 0.0
  init_yaw = 0.0

  # Parameters
  config = os.path.join(
      get_package_share_directory('challenge6'),
      'config',
      'params.yaml'
      )
  
  path_generator_node = Node(
      package='challenge6',
      executable='path_generator',
      output='screen',
      parameters = [config]
  )
    
  controller_node = Node(name="controller",
                          package='challenge6',
                          executable='controller',
                          parameters=[{
                              'init_pose_x': init_x,
                              'init_pose_y': init_y,
                              'init_pose_yaw': init_yaw,
                              'algorithm': 'bug2'
                          }]
                          )
  
  odometry_node = Node(name="odometry",
                          package='challenge6',
                          executable='odometry',
                          parameters=[{
                              'init_pose_x': init_x,
                              'init_pose_y': init_y,
                              'init_pose_yaw': init_yaw,
                              'use_linear_model': False,
                          }]
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
      path_generator_node,
      shutdown_log,
      ])

  return l_d