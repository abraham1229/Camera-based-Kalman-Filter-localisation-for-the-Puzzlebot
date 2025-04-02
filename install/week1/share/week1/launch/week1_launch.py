import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

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