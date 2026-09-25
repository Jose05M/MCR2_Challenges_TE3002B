import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = FindPackageShare('yolov8_ros2')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=PathJoinSubstitution([pkg, 'config', 'params.yaml']),
        description='Path to YAML parameter file',
    )

    config_file = LaunchConfiguration('config')

    detector_semaforo = Node(
        package='yolov8_ros2',
        executable='traffic_light_detector',
        name='traffic_light_detector',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )

    detector_yolo = Node(
        package='yolov8_ros2',
        executable='yolov8_recognition',
        name='yolov8_recognition',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        config_arg,
        detector_semaforo,
        detector_yolo,
    ])
