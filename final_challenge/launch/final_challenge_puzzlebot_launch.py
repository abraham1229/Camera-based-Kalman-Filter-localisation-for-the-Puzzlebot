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
      get_package_share_directory('final_challenge'),
      'config',
      'params.yaml'
      )
  
  path_generator_node = Node(
      package='final_challenge',
      executable='path_generator',
      output='screen',
      parameters=[{
            'init_pose_x': init_x,
            'init_pose_y': init_y,
            'init_pose_yaw': init_yaw,
            'num_goals': 4,
            'x_goal_1': 1.0,
            'y_goal_1': -2.0,
            'x_goal_2': 4.0,
            'y_goal_2': -2.0,
            'x_goal_3': 2.0,
            'y_goal_3': 2.0,
            'x_goal_4': 0.0,
            'y_goal_4': 0.0
        }]
  )
    
  controller_node = Node(name="controller",
                          package='final_challenge',
                          executable='controller',
                          parameters=[{
                              'init_pose_x': init_x,
                              'init_pose_y': init_y,
                              'init_pose_yaw': init_yaw
                          }]
                          )
  
  odometry_node = Node(name="odometry",
                          package='final_challenge',
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