"""Line following that obeys a traffic light.

The line detector publishes the offset of the black line on /line_error, the
traffic light detector publishes /traffic_light/state, and the line follower
controller (PD) drives the robot along the line, stopping on red.

How to run:
    1. Start the micro-ROS agent and the camera on the Jetson (see teleop.launch.py).
    2. Place the robot on the line, then on the PC:
           ros2 launch pzb_bringup line_follower.launch.py

    To test without the robot camera, publish the laptop webcam on
    /video_source/raw in another terminal:
           ros2 run pzb_vision usb_camera_publisher

What this launch actually executes:
    ros2 run pzb_vision line_detector --ros-args --params-file line_follower.yaml
    ros2 run pzb_vision traffic_light_detector --ros-args --params-file line_follower.yaml
    ros2 run pzb_control line_follower_controller --ros-args --params-file line_follower.yaml
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
        default_value=PathJoinSubstitution([pkg, 'config', 'line_follower.yaml']),
        description='Path to YAML parameter file',
    )

    config_file = LaunchConfiguration('config')

    detector_node = Node(
        package='pzb_vision',
        executable='traffic_light_detector',
        name='traffic_light_detector',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )

    controller_node = Node(
        package='pzb_control',
        executable='line_follower_controller',
        name='line_follower_controller',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )

    line_node = Node(
        package='pzb_vision',
        executable='line_detector',
        name='line_detector',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        config_arg,
        line_node,
        detector_node,
        controller_node,
    ])
