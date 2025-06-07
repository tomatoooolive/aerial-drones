from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    drones = [
        {
            'name': 'leader',
            'urdf': 'leader_tello.urdf',
            'position': ('0', '0', '1'),
            'yaw': '0'
        },
        {
            'name': 'follower',
            'urdf': 'follower_tello.urdf',
            'position': ('1', '0', '1'),
            'yaw': '1.57'
        }
    ]

    world_path = os.path.join(
        get_package_share_directory('tello_gazebo'),
        'worlds',
        'simple.world'
    )

    ld = LaunchDescription()

    # Launch Gazebo only once
    ld.add_action(ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_path
        ],
        output='screen'
    ))

    for drone in drones:
        urdf_path = os.path.join(
            get_package_share_directory('tello_description'),
            'urdf',
            drone['urdf']
        )
        x, y, z = drone['position']
        yaw = drone['yaw']
        name = drone['name']

        ld.add_action(Node(
            package='tello_gazebo',
            executable='inject_entity.py',
            output='screen',
            arguments=[
                urdf_path, x, y, z, yaw, name
            ]
        ))

        ld.add_action(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[urdf_path]
        ))

        ld.add_action(Node(
            package='joy',
            executable='joy_node',
            output='screen',
            namespace=name
        ))

        ld.add_action(Node(
            package='tello_driver',
            executable='tello_joy_main',
            output='screen',
            namespace=name
        ))

    return ld
