# """Simulate a Tello drone"""

# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import ExecuteProcess


# def generate_launch_description():
#     ns = 'drone1'
#     world_path = os.path.join(get_package_share_directory('tello_gazebo'), 'worlds', 'simple.world')
#     urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'follower_tello.urdf')

#     return LaunchDescription([
#         # Launch Gazebo, loading tello.world
#         ExecuteProcess(cmd=[
#             'gazebo',
#             '--verbose',
#             '-s', 'libgazebo_ros_init.so',  # Publish /clock
#             '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node
#             world_path
#         ], output='screen'),

#         # Spawn tello.urdf
#         Node(package='tello_gazebo', executable='inject_entity.py', output='screen',
#              arguments=[urdf_path, '0', '0', '1', '1.57079632679']),

#         # Publish static transforms
#         Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
#              arguments=[urdf_path]),

#         # Joystick driver, generates /namespace/joy messages
#         Node(package='joy', executable='joy_node', output='screen',
#              namespace=ns),

#         # Joystick controller, generates /namespace/cmd_vel messages
#         Node(package='tello_driver', executable='tello_joy_main', output='screen',
#              namespace=ns),
#     ])

## use this command to launch 
# -> ros2 launch tello_gazebo simple_launch.py urdf_name:=follower_tello.urdf namespace:=follower


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_name = LaunchConfiguration('urdf_name')
    namespace = LaunchConfiguration('namespace')

    urdf_path = PathJoinSubstitution([
        get_package_share_directory('tello_description'),
        'urdf',
        LaunchConfiguration('urdf_name')
    ])
    world_path = os.path.join(
        get_package_share_directory('tello_gazebo'),
        'worlds',
        'simple.world'
    )

    return LaunchDescription([
        DeclareLaunchArgument('urdf_name', default_value='tello.urdf', description='URDF file name'),
        DeclareLaunchArgument('namespace', default_value='solo', description='Namespace for the drone'),

        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
                world_path
            ],
            output='screen'
        ),

        Node(
            package='tello_gazebo',
            executable='inject_entity.py',
            output='screen',
            arguments=[
                 urdf_path,
                 '0', '0', '1', '1.57079632679',
                 LaunchConfiguration('namespace')  # ← This becomes the model name
            ]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[urdf_path]
        ),

        Node(
            package='joy',
            executable='joy_node',
            output='screen',
            namespace=namespace
        ),

        Node(
            package='tello_driver',
            executable='tello_joy_main',
            output='screen',
            namespace=namespace
        )
    ])
