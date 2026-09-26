"""Closed-loop point-to-point navigation.

Wheel odometry closes the loop: the path generator sends one goal at a time on
/goal and the closed-loop controller reaches it using /odom feedback.

    part:=1  2 m square (config/closed_loop_part1.yaml)
    part:=2  goals with final heading (config/closed_loop_part2.yaml)

How to run:
    1. Start the micro-ROS agent on the Jetson (see teleop.launch.py).
    2. Place the robot at (0, 0) facing +x, then on the PC:
           ros2 launch pzb_bringup closed_loop.launch.py part:=1

What this launch actually executes:
    ros2 run pzb_odometry odometry --ros-args --params-file puzzlebot.yaml
    ros2 run pzb_control closed_loop_path_generator --ros-args --params-file closed_loop_part<N>.yaml
    ros2 run pzb_control closed_loop_controller --ros-args --params-file closed_loop_part<N>.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_dir = PathJoinSubstitution([FindPackageShare('pzb_bringup'), 'config'])

    part_arg = DeclareLaunchArgument(
        'part', default_value='1', choices=['1', '2'],
        description='1 = 2 m square, 2 = goals with final heading')

    robot_config = PathJoinSubstitution([config_dir, 'puzzlebot.yaml'])
    part_config = PathJoinSubstitution(
        [config_dir, ['closed_loop_part', LaunchConfiguration('part'), '.yaml']])

    odometry = Node(
        package='pzb_odometry',
        executable='odometry',
        parameters=[robot_config],
        output='screen',
        emulate_tty=True,
    )

    path_generator = Node(
        package='pzb_control',
        executable='closed_loop_path_generator',
        parameters=[part_config],
        output='screen',
        emulate_tty=True,
    )

    controller = Node(
        package='pzb_control',
        executable='closed_loop_controller',
        parameters=[part_config],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([part_arg, odometry, controller, path_generator])
