import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_robot_group(robot_index, robot_desc, path_type):
    namespace = f'group{robot_index}'

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'robot_state_publisher_{robot_index}',
        output='screen',
        parameters=[{'frame_prefix': f'{namespace}/','robot_description': robot_desc}],
        namespace=namespace
    )

    puzzlebot_node = Node(
        name='puzzlebot',
        package='challenge4',
        executable='puzzlebot',
        namespace=namespace,
        parameters=[{
            'init_pose_x': 1.0 * robot_index,
            'init_pose_y': 1.0,
            'init_pose_yaw': 1.57,
            'odom_frame': 'odom'
        }]
    )

    controller_node = Node(
        name='controller',
        package='challenge4',
        executable='controller',
        namespace=namespace,
        parameters=[{
            'init_pose_x': 1.0,
            'init_pose_y': 1.0 * robot_index,
            'init_pose_yaw': 1.57
        }]
    )

    odometry_node = Node(
        name='odometry',
        package='challenge4',
        executable='odometry',
        namespace=namespace,
        parameters=[{
            'init_pose_x': 1.0 ,
            'init_pose_y': 1.0 * robot_index,
            'init_pose_yaw': 1.57
        }]
    )

    path_generator_node = Node(
        name='path_generator',
        package='challenge4',
        executable='path_generator',
        namespace=namespace,
        parameters=[{
            'init_pose_x': 1.0,
            'init_pose_y': 1.0 * robot_index,
            'init_pose_yaw': 1.57,
            'type': path_type
        }]
    )

    return [robot_state_pub, puzzlebot_node, controller_node, odometry_node, path_generator_node]



def generate_launch_description():
    
    urdf_file_name = 'puzzle_mesh.urdf'
    urdf = os.path.join(
        get_package_share_directory('challenge4'),
        'urdf',
        urdf_file_name)
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # Número de robots
    num_robots = 3  

    robot_nodes = []
    for i in range(1, num_robots + 1):
        robot_nodes.extend(generate_robot_group(i, robot_desc,i))

    # Agregar RViz
    rviz_config = os.path.join(
        get_package_share_directory('challenge4'),
        'rviz',
        'multi_puzzledrone.rviz'
    )

    rviz_node = Node(
        name='rviz',
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription(robot_nodes + [rviz_node])
