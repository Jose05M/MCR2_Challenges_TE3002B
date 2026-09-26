"""Mini Challenge AI: traffic sign detection with YOLOv8 (plus the traffic light).

The traffic sign detector runs the trained YOLOv8 model (pzb_detection/models)
on the camera image and publishes the closest sign on /traffic_signals/state;
the traffic light detector publishes /traffic_light/state.

How to run:
    1. Start the micro-ROS agent and the camera on the Jetson (see teleop.launch.py).
    2. On the PC:
           ros2 launch pzb_bringup sign_detection.launch.py

    To test without the robot camera, publish the laptop webcam on
    /video_source/raw in another terminal:
           ros2 run pzb_vision usb_camera_publisher

What this launch actually executes:
    ros2 run pzb_vision traffic_light_detector --ros-args --params-file sign_detection.yaml
    ros2 run pzb_detection traffic_sign_detector --ros-args --params-file sign_detection.yaml
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
        default_value=PathJoinSubstitution([pkg, 'config', 'sign_detection.yaml']),
        description='Path to YAML parameter file',
    )

    config_file = LaunchConfiguration('config')

    detector_semaforo = Node(
        package='pzb_vision',
        executable='traffic_light_detector',
        name='traffic_light_detector',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )

    detector_yolo = Node(
        package='pzb_detection',
        executable='traffic_sign_detector',
        name='traffic_sign_detector',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        config_arg,
        detector_semaforo,
        detector_yolo,
    ])
