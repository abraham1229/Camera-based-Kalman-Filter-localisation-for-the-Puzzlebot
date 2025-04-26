import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file_name = 'puzzle_mesh.urdf'
    urdf = os.path.join(
        get_package_share_directory('challenge4'),
        'urdf',
        urdf_file_name)
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()


    # Robot 1: group1

    robot1_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'frame_prefix': 'group1/','robot_description': robot_desc}],
        namespace='group1'
    )

    robot1_node = Node(
        name='puzzlebot',
        package='challenge4',
        executable='puzzlebot',
        namespace='group1',
        parameters=[{
                    'init_pose_x':2.0,
                    'init_pose_y': 2.0,
                    'init_pose_z': 1.0,
                    'init_pose_yaw': 1.57,
                    'init_pose_pitch': 0.0,
                    'init_pose_roll': 0.0,
                    'odom_frame':'odom'
                }]
            )   


    # Robot 2: group2
    
    robot2_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'frame_prefix': 'group2/','robot_description': robot_desc}],
        namespace='group2'
    )

    robot2_node = Node(
        name='puzzlebot',
        package='challenge4',
        executable='puzzlebot',
        namespace='group2',
        parameters=[{
                    'init_pose_x':-2.0,
                    'init_pose_y': 2.0,
                    'init_pose_z': 1.0,
                    'init_pose_yaw': 1.57,
                    'init_pose_pitch': 0.0,
                    'init_pose_roll': 0.0,
                    'odom_frame':'odom'
                }]
            )   
    
    rviz_config = os.path.join(
                            get_package_share_directory('challenge4'),
                            'rviz',
                            'multi_puzzledrone.rviz'
                            )
    
    rviz_node = Node(name='rviz',
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', rviz_config],
                    )

    
    return LaunchDescription([
        robot1_state_pub,
        robot1_node,
        robot2_state_pub,
        robot2_node,
        rviz_node
    ])