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
        get_package_share_directory('challenge3'),
        'config',
        'params.yaml'
        )
    
    path_generator_node = Node(
        package='challenge3',
        executable='path_generator',
        output='screen',
        parameters = [config]
    )
    


    urdf_file_name = 'puzzle_mesh.urdf'
    urdf = os.path.join(
        get_package_share_directory('challenge3'),
        'urdf',
        urdf_file_name)
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()


    static_transform_node = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                arguments = ['--x', '2', '--y', '1', '--z', '0.0',
                                            '--yaw', '0.0', '--pitch', '0', '--roll', '0.0',
                                            '--frame-id', 'map', '--child-frame-id', 'odom']
                                )

    static_transform_node_2 = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                arguments = ['--x', '0', '--y', '0', '--z', '0.0',
                                            '--yaw', '0.0', '--pitch', '0', '--roll', '0.0',
                                            '--frame-id', 'world', '--child-frame-id', 'map']
                                )

    robot_state_pub_node = Node(
                            package='robot_state_publisher',
                            executable='robot_state_publisher',
                            name='robot_state_publisher',
                            output='screen',
                            parameters=[{'robot_description': robot_desc}],
                            arguments=[urdf]
                            )
    
    rviz_config = os.path.join(
                            get_package_share_directory('challenge3'),
                            'rviz',
                            'puzzlebot.rviz'
                            )
    
    rviz_node = Node(name='rviz',
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', rviz_config],
                    )
    
    rqt_tf_tree_node = Node(name='rqt_tf_tree',
                    package='rqt_tf_tree',
                    executable='rqt_tf_tree'
                    )
    
    puzzlebot_node = Node(name="puzzlebot",
                            package='challenge3',
                            executable='puzzlebot'
                            )
    
    controller_node = Node(name="controller",
                            package='challenge3',
                            executable='controller'
                            )
    
    odometry_node = Node(name="odometry",
                            package='challenge3',
                            executable='odometry'
                            )

    shutdown_on_exit = [RegisterEventHandler(
                            OnProcessExit(
                                target_action=node,
                                on_exit=[
                                    LogInfo(msg=(EnvironmentVariable(name='USER'),
                                            ' exited the node')),
                                    EmitEvent(event=Shutdown(
                                        reason='Node Exited'))
                                ]
                            )
                        ) for node in [robot_state_pub_node, puzzlebot_node]
                    ]
 

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
        static_transform_node, 
        static_transform_node_2,
        robot_state_pub_node, 
        rviz_node, 
        # rqt_tf_tree_node,
        puzzlebot_node,
        controller_node,
        odometry_node,
        shutdown_log,
        *shutdown_on_exit,
        path_generator_node
        ])

    return l_d