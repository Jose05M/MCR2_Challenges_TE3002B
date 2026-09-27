"""Final challenge, laptop side: line detector, odometry, FSM and controller.

The YOLO sign detector runs in its own terminal (puzzlebot_yolo.launch.py).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = FindPackageShare('pzb_bringup')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=PathJoinSubstitution([pkg, 'config', 'puzzlebot_final.yaml']),
        description='Path to YAML parameter file',
    )

    config_file = LaunchConfiguration('config')
    robot_config = PathJoinSubstitution([pkg, 'config', 'puzzlebot.yaml'])

    line_detector_node = Node(
        package='pzb_vision',
        executable='line_detector',
        name='line_detector',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )

    controller_node = Node(
        package='pzb_control',
        executable='puzzlebot_controller',
        name='puzzlebot_controller',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )

    fsm_node = Node(
        package='pzb_fsm',
        executable='fsm_node',
        name='fsm_node',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )

    odometry_node = Node(
        package='pzb_odometry',
        executable='odometry',
        name='odometry',
        parameters=[robot_config],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        config_arg,
        controller_node,
        fsm_node,
        odometry_node,
        line_detector_node,
    ])
