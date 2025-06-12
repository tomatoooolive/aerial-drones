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
            'position': ('3', '0', '1'),
            'yaw': '1.57'
        }
    ]

    world_path = os.path.join(
        get_package_share_directory('tello_gazebo'),
        'worlds',
        'simple.world'
    )

    ld = LaunchDescription()

    # 1) Launch Gazebo once
    ld.add_action(ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_path
        ],
        output='screen'
    ))

    # 2) Spawn each drone
    for drone in drones:
        urdf_path = os.path.join(
            get_package_share_directory('tello_description'),
            'urdf',
            drone['urdf']
        )
        x, y, z = drone['position']
        yaw = drone['yaw']
        name = drone['name']

        # Inject into Gazebo
        ld.add_action(Node(
            package='tello_gazebo',
            executable='inject_entity.py',
            output='screen',
            arguments=[urdf_path, x, y, z, yaw, name]
        ))

        # Publish its TF
        ld.add_action(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[urdf_path]
        ))

        # Joystick and driver under the drone’s namespace
        ld.add_action(Node(
            package='joy',
            executable='joy_node',
            namespace=name,
            output='screen'
        ))
        ld.add_action(Node(
            package='tello_driver',
            executable='tello_joy_main',
            namespace=name,
            output='screen'
        ))

    # 3) Exactly one follower_handshake under /follower
    ld.add_action(Node(
        package='tello_comm',
        executable='follower_handshake',
        namespace='follower',
        output='screen'
    ))
    # 4) Exactly one leader_handshake in the global namespace
    ld.add_action(Node(
        package='tello_comm',
        executable='leader_handshake',
        output='screen'
    ))

    return ld
