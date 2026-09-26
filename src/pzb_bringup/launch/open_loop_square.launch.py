"""Open-loop square path.

Drives the Puzzlebot through the waypoints in config/open_loop_square.yaml
(2 m square) without feedback: the path generator sends one point at a time on
/pose and the open-loop controller reaches it with a timed turn + straight FSM.

How to run:
    1. Start the micro-ROS agent on the Jetson (see teleop.launch.py).
    2. Place the robot at (0, 0) facing +x, then on the PC:
           ros2 launch pzb_bringup open_loop_square.launch.py

What this launch actually executes:
    ros2 run pzb_control open_loop_path_generator --ros-args --params-file open_loop_square.yaml
    ros2 run pzb_control open_loop_controller --ros-args --params-file open_loop_square.yaml
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('pzb_bringup'), 'config', 'open_loop_square.yaml'])

    path_generator = Node(
        package='pzb_control',
        executable='open_loop_path_generator',
        parameters=[config],
        output='screen',
        emulate_tty=True,
    )

    controller = Node(
        package='pzb_control',
        executable='open_loop_controller',
        parameters=[config],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([controller, path_generator])
