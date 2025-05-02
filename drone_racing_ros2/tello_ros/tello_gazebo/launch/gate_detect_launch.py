"""Launch a custom world with a Tello drone and joystick control"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='drone1',
        description='ROS namespace for the Tello drone'
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory('tello_gazebo'),
            'worlds', 'four_green_gates.world'
        ),
        description='Path to the Gazebo world file to load'
    )

    # Substitutions
    ns = LaunchConfiguration('namespace')
    world_file = LaunchConfiguration('world')
    urdf_file = os.path.join(
        get_package_share_directory('tello_description'),
        'urdf', 'tello.urdf'
    )

    return LaunchDescription([
        namespace_arg,
        world_arg,

        # Launch Gazebo with the specified world
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose',
                '-s', 'libgazebo_ros_init.so',     # Clock publisher
                '-s', 'libgazebo_ros_factory.so',  # Factory for spawn
                world_file
            ],
            output='screen'
        ),

        # Spawn the Tello model
        Node(
            package='tello_gazebo', executable='inject_entity.py', output='screen',
            arguments=[urdf_file, '0', '0', '1', '1.5708'],
            namespace=ns
        ),

        # Publish robot state (TF) from URDF
        Node(
            package='robot_state_publisher', executable='robot_state_publisher', output='screen',
            arguments=[urdf_file],
            namespace=ns
        ),

        # Joystick input
        Node(
            package='joy', executable='joy_node', output='screen',
            namespace=ns
        ),

        # Joystick teleop for Tello
        Node(
            package='tello_driver', executable='tello_joy_main', output='screen',
            namespace=ns
        ),
    ])
