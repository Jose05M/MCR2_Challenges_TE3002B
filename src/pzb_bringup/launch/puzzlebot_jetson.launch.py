"""Final challenge, Jetson side: traffic light detector + image compressor."""

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

    traffic_light_node = Node(
        package='pzb_vision',
        executable='traffic_light_detector',
        name='traffic_light_detector',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )

    image_compressor_node = Node(
        package='pzb_vision',
        executable='image_compressor',
        name='image_compressor',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        config_arg,
        traffic_light_node,
        image_compressor_node,
    ])
