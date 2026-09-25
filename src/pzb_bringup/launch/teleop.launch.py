"""Week 1 — Getting to know the Puzzlebot: keyboard teleoperation.

Drives the Puzzlebot with teleop_twist_keyboard on /cmd_vel and plots the
commanded velocity against the wheel encoder speeds, to find the maximum and
minimum linear/angular speeds the robot can handle.

How to run:
    1. Connect the PC to the Jetson hotspot (Wi-Fi network of the Puzzlebot).
    2. On the Jetson, over SSH, start the micro-ROS agent:
           ssh puzzlebot@10.42.0.1
           ros2 launch puzzlebot_ros micro_ros_agent.launch.py
    3. On the PC (teleop_twist_keyboard installed with
       `sudo apt install ros-humble-teleop-twist-keyboard`):
           ros2 launch pzb_bringup teleop.launch.py

What this launch actually executes:
    gnome-terminal --wait -- ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ros2 run rqt_plot rqt_plot /cmd_vel/linear/x /cmd_vel/angular/z \
        /VelocityEncR/data /VelocityEncL/data
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # teleop_twist_keyboard reads stdin, which ros2 launch does not forward,
    # so it runs in its own terminal window.
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        prefix='gnome-terminal --wait --',
    )

    plotter = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_plot', 'rqt_plot',
             '/cmd_vel/linear/x', '/cmd_vel/angular/z',
             '/VelocityEncR/data', '/VelocityEncL/data'],
    )

    return LaunchDescription([teleop, plotter])
