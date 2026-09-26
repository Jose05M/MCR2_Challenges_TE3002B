"""Half-term: waypoint navigation that obeys a traffic light.

The traffic light detector publishes RED / YELLOW / GREEN / UNKNOWN from the
camera, and the navigation controller scales its speed accordingly while it
drives through the waypoints in config/traffic_light_nav.yaml (odometry feedback).

How to run:
    1. Start the micro-ROS agent and the camera on the Jetson (see teleop.launch.py).
    2. Place the robot at (0, 0) facing +x, then on the PC:
           ros2 launch pzb_bringup traffic_light_nav.launch.py
       Add record:=true to also log a CSV for pzb_tools/scripts/graphs.py.

What this launch actually executes:
    ros2 run pzb_odometry odometry --ros-args --params-file puzzlebot.yaml
    ros2 run pzb_vision traffic_light_detector --ros-args --params-file traffic_light_nav.yaml
    ros2 run pzb_control traffic_light_nav_controller
        --ros-args --params-file traffic_light_nav.yaml
    ros2 run pzb_tools analysis_node                    (only with record:=true)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = FindPackageShare('pzb_bringup')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=PathJoinSubstitution([pkg, 'config', 'traffic_light_nav.yaml']),
        description='Path to YAML parameter file',
    )

    record_arg = DeclareLaunchArgument(
        'record',
        default_value='false',
        description='Also run analysis_node to log a CSV of the run',
    )

    config_file = LaunchConfiguration('config')
    robot_config = PathJoinSubstitution([pkg, 'config', 'puzzlebot.yaml'])

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
        executable='traffic_light_nav_controller',
        name='traffic_light_nav_controller',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )

    odom_node = Node(
        package='pzb_odometry',
        executable='odometry',
        name='odometry',
        parameters=[robot_config],
        output='screen',
        emulate_tty=True,
    )

    analysis_node = Node(
        package='pzb_tools',
        executable='analysis_node',
        name='analysis_node',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('record')),
    )

    return LaunchDescription([
        config_arg,
        record_arg,
        odom_node,
        detector_node,
        controller_node,
        analysis_node,
    ])
