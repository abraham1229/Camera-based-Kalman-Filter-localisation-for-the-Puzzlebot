import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_robot_group(robot_index, robot_desc, init_x, init_y, init_yaw):
    namespace = f'group{robot_index}'

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'robot_state_publisher_{robot_index}',
        output='screen',
        parameters=[{'frame_prefix': f'{namespace}/', 'robot_description': robot_desc}],
        namespace=namespace
    )

    puzzlebot_node = Node(
        name='puzzlebot',
        package='challenge4',
        executable='puzzlebot',
        namespace=namespace,
        parameters=[{
            'init_pose_x': init_x,
            'init_pose_y': init_y,
            'init_pose_yaw': init_yaw,
            'odom_frame': 'odom'
        }]
    )

    controller_node = Node(
        name='controller',
        package='challenge4',
        executable='controller',
        namespace=namespace,
        parameters=[{
            'init_pose_x': init_x,
            'init_pose_y': init_y,
            'init_pose_yaw': init_yaw
        }]
    )

    odometry_node = Node(
        name='odometry',
        package='challenge4',
        executable='odometry',
        namespace=namespace,
        parameters=[{
            'init_pose_x': init_x,
            'init_pose_y': init_y,
            'init_pose_yaw': init_yaw
        }]
    )

    path_generator_node = Node(
        name='path_generator',
        package='challenge4',
        executable='path_generator',
        namespace=namespace,
        parameters=[{
            'init_pose_x': init_x,
            'init_pose_y': init_y,
            'init_pose_yaw': init_yaw,
            'type': 0
        }]
    )

    return [robot_state_pub, puzzlebot_node, controller_node, odometry_node, path_generator_node]


def generate_launch_description():
    urdf_file_name = 'puzzle_mesh.urdf'
    urdf = os.path.join(
        get_package_share_directory('challenge4'),
        'urdf',
        urdf_file_name
    )

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # Definir posiciones iniciales manualmente (x, y, yaw)
    robots_positions = [
        (-2.0, 2.0, 1.57),  # Robot 1
    ]

    robot_nodes = []
    for idx, (x, y, yaw) in enumerate(robots_positions, start=1):
        robot_nodes.extend(generate_robot_group(idx, robot_desc, x, y, yaw))

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
