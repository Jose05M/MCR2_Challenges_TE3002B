"""Final challenge, laptop side: YOLO traffic sign detector."""

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

    sign_detector_node = Node(
        package='pzb_detection',
        executable='traffic_sign_detector',
        name='traffic_sign_detector',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        config_arg,
        sign_detector_node,
    ])
