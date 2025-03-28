import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    #urdf_file_name = 'puzzle.urdf'
    #urdf = os.path.join(
    #    get_package_share_directory('week1'),
    #    'urdf',
    #    urdf_file_name)
    #
    #with open(urdf, 'r') as infp:
    #    robot_desc = infp.read()


    #robot_state_pub_node = Node(
    #                        package='robot_state_publisher',
    #                        executable='robot_state_publisher',
    #                        name='robot_state_publisher',
    #                        output='screen',
    #                        parameters=[{'robot_description': robot_desc}],
    #                        arguments=[urdf]
    #                        )
    
    puzzle_nodes = Node(
                    name="puzzlebot",
                    package='week1',
                    executable='puzzlebot'
                    )

    rviz_node = Node(name='rviz',
                    package='rviz2',
                    executable='rviz2'
                    )
    
    rqt_tf_tree_node = Node(name='rqt_tf_tree',
                    package='rqt_tf_tree',
                    executable='rqt_tf_tree'
                    )

    l_d = LaunchDescription([puzzle_nodes,rviz_node,rqt_tf_tree_node])

    return l_d