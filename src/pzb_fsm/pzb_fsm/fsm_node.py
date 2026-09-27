#!/usr/bin/env python3
"""
Puzzlebot FSM (final challenge decision layer).

Decides WHAT the robot does from the traffic light, traffic signs and
intersection (zebra crossing) detections, and sends it as a MotionCommand on
/motion_command. It never publishes velocities: pzb_control/puzzlebot_controller
executes the commands (line-following PD and odometry manoeuvres) and reports
the end of DRIVE / ROTATE on /motion_done.

Modes: FOLLOW, WAIT_INTERSECTION, TURN, STRAIGHT, SPECIAL (GIVE_WAY, WORKERS,
ROUND) and STOPPED (resume with
`ros2 topic pub --once /fsm_command std_msgs/msg/String "{data: 'START'}"`).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from pzb_interfaces.msg import MotionCommand


class PuzzlebotFSM(Node):
    """Traffic light / sign / intersection state machine."""

    def __init__(self):
        super().__init__('fsm_node')

        self.declare_parameter('straight_dist', 0.45)       # [m] crossing an intersection
        self.declare_parameter('straight_speed', 0.10)      # [m/s]
        self.declare_parameter('special_duration', 8.0)     # [s] WORKERS / ROUND
        self.declare_parameter('special_slow_after', 4.0)   # [s] slow down after this
        self.declare_parameter('stop_duration', 1.5)        # [s] wait at the zebra crossing
        self.declare_parameter('give_way_wait', 2.0)        # [s] GIVE_WAY stop
        self.declare_parameter('stop_sign_delay', 6.0)      # [s] keep following before STOP
        self.declare_parameter('turn_forward_dist', 0.40)   # [m] before rotating
        self.declare_parameter('turn_forward_speed', 0.10)  # [m/s]
        self.declare_parameter('turn_extra_angle', np.pi / 13)  # [rad] on top of 90°
        self.declare_parameter('turn_exit_dist', 0.10)      # [m] after rotating
        self.declare_parameter('turn_exit_speed', 0.08)     # [m/s]

        self.declare_parameter('state_topic', '/traffic_light/state')
        self.declare_parameter('signal_topic', '/traffic_sign/state')
        self.declare_parameter('activate_cmd', '/fsm_command')
        self.declare_parameter('intersection_topic', '/intersection_detected')
        self.declare_parameter('motion_command_topic', '/motion_command')
        self.declare_parameter('motion_done_topic', '/motion_done')

        self.straight_distance = self.get_parameter('straight_dist').value
        self.straight_speed = self.get_parameter('straight_speed').value
        self.special_duration = self.get_parameter('special_duration').value
        self.special_slow_after = self.get_parameter('special_slow_after').value
        self.stop_duration = self.get_parameter('stop_duration').value
        self.give_way_wait = self.get_parameter('give_way_wait').value
        self.stop_sign_delay = self.get_parameter('stop_sign_delay').value
        self.turn_forward_dist = self.get_parameter('turn_forward_dist').value
        self.turn_forward_speed = self.get_parameter('turn_forward_speed').value
        self.turn_extra_angle = self.get_parameter('turn_extra_angle').value
        self.turn_exit_dist = self.get_parameter('turn_exit_dist').value
        self.turn_exit_speed = self.get_parameter('turn_exit_speed').value

        state_topic = self.get_parameter('state_topic').value
        sign_topic = self.get_parameter('signal_topic').value
        activate_topic = self.get_parameter('activate_cmd').value
        intersection_topic = self.get_parameter('intersection_topic').value
        command_topic = self.get_parameter('motion_command_topic').value
        done_topic = self.get_parameter('motion_done_topic').value

        # Perception
        self.tl_state = 'UNKNOWN'
        self.sign_state = "NONE"
        self.zebra_crossing = False
        self.last_tl_log = ""

        # FSM modes: FOLLOW, WAIT_INTERSECTION, TURN, STRAIGHT, SPECIAL, STOPPED
        self.mode = "FOLLOW"
        self.turn_phase = "NONE"   # FORWARD, ROTATE, EXIT

        self.pending_action = "NONE"
        self.special_behavior = "NONE"
        self.special_start_time = None

        # Zebra crossing stop
        self.intersection_lock = False
        self.stop_start_time = None

        # Set by /motion_done when the controller finishes a DRIVE / ROTATE
        self.motion_done = False

        # ROS I/O
        self.pub_cmd = self.create_publisher(MotionCommand, command_topic, 10)
        self.sub_tl = self.create_subscription(String, state_topic, self._tl_callback, 10)
        self.sub_sign = self.create_subscription(String, sign_topic, self._sign_callback, 10)
        self.sub_fsm = self.create_subscription(
            String, activate_topic, self._fsm_command_callback, 10)
        self.sub_intersection = self.create_subscription(
            Bool, intersection_topic, self._intersection_callback, 10)
        self.sub_done = self.create_subscription(Bool, done_topic, self._done_callback, 10)

        # Decision loop at 20 Hz (same rate as the controller)
        self.timer = self.create_timer(0.05, self._fsm_loop)
        self.get_logger().info('PuzzlebotFSM Started')

    # ------------------------------------------------------------------
    # Commands sent to the controller
    # ------------------------------------------------------------------
    @staticmethod
    def follow(linear_scale=1.0, angular_scale=1.0):
        return MotionCommand(mode=MotionCommand.FOLLOW_LINE,
                             linear_scale=float(linear_scale),
                             angular_scale=float(angular_scale))

    @staticmethod
    def stop():
        return MotionCommand(mode=MotionCommand.STOP)

    @staticmethod
    def drive(distance, speed):
        return MotionCommand(mode=MotionCommand.DRIVE,
                             distance=float(distance), speed=float(speed))

    @staticmethod
    def rotate(angle):
        return MotionCommand(mode=MotionCommand.ROTATE, angle=float(angle))

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _tl_callback(self, msg: String):
        self.tl_state = msg.data

    def _intersection_callback(self, msg):
        self.zebra_crossing = msg.data

    def _done_callback(self, msg):
        if msg.data:
            self.motion_done = True

    def _fsm_command_callback(self, msg):
        command = msg.data
        if (self.mode == "STOPPED" and command == "START"):
            self.pending_action = "NONE"
            self.mode = "FOLLOW"
            self.get_logger().info("Exiting STOPPED mode")

    def _sign_callback(self, msg):
        self.sign_state = msg.data

        # store the pending action
        if self.sign_state in ["LEFT", "RIGHT", "STRAIGHT", "STOP"]:
            self.pending_action = self.sign_state

        elif self.sign_state in ["ROUND", "GIVE_WAY", "WORKERS"]:
            self.special_behavior = self.sign_state

    # ------------------------------------------------------------------
    # Transitions
    # ------------------------------------------------------------------
    def update_fsm(self):
        # STOP
        if self.pending_action == "STOP" and self.mode != "STOPPED":
            self.mode = "STOPPED"
            self.stop_start_time = (self.current_time)
            self.get_logger().info("Entering STOPPED mode")
            return

        # FOLLOW -> WAIT_INTERSECTION
        if (self.mode == "FOLLOW" and self.zebra_crossing and not self.intersection_lock
                and self.pending_action in ["LEFT", "RIGHT", "STRAIGHT"]):
            self.intersection_lock = True
            self.mode = "WAIT_INTERSECTION"
            self.stop_start_time = self.current_time
            self.get_logger().info("Zebra crossing detected")
            return

        # FOLLOW -> SPECIAL
        elif (self.mode == "FOLLOW" and self.special_behavior != "NONE"):
            self.mode = "SPECIAL"
            self.special_start_time = (self.current_time)
            self.get_logger().info(f"Entering SPECIAL mode: {self.special_behavior}")
            return

    # ------------------------------------------------------------------
    # Modes: each one returns the MotionCommand for this cycle
    # ------------------------------------------------------------------
    def handle_follow_mode(self):
        if self.tl_state == "RED":
            # the PD keeps running, output forced to 0
            return self.follow(0.0, 0.0)
        elif self.tl_state == "YELLOW":
            return self.follow(0.5, 1.0)
        return self.follow(1.0, 1.0)

    def handle_wait_intersection(self):
        elapsed = (self.current_time - self.stop_start_time)
        # never move on while the light is red
        if self.tl_state == "RED":
            return self.stop()

        if elapsed >= self.stop_duration:
            if self.pending_action in ["LEFT", "RIGHT"]:
                self.mode = "TURN"
                self.turn_phase = "FORWARD"

            elif self.pending_action == "STRAIGHT":
                self.mode = "STRAIGHT"

            self.motion_done = False
            self.get_logger().info(f"Executing: {self.pending_action}")

        return self.stop()

    def handle_turn_mode(self):
        if self.turn_phase == "FORWARD":
            # move forward before turning
            if not self.motion_done:
                return self.drive(self.turn_forward_dist, self.turn_forward_speed)
            self.motion_done = False
            self.turn_phase = "ROTATE"
            self.get_logger().info("Starting rotation")

        if self.turn_phase == "ROTATE":
            angle = np.pi / 2 + self.turn_extra_angle
            if self.pending_action == "RIGHT":
                angle = -angle
            if not self.motion_done:
                return self.rotate(angle)
            self.motion_done = False
            self.turn_phase = "EXIT"
            self.get_logger().info("Rotation completed")

        if self.turn_phase == "EXIT":
            # distance after the turn
            if not self.motion_done:
                return self.drive(self.turn_exit_dist, self.turn_exit_speed)
            self.motion_done = False
            self.pending_action = "NONE"
            self.mode = "FOLLOW"
            self.turn_phase = "NONE"
            self.zebra_crossing = False
            self.intersection_lock = False
            self.get_logger().info("Turn completed")

        return self.stop()

    def handle_straight_mode(self):
        # wait while red (the controller pauses the DRIVE and resumes it)
        if self.tl_state == "RED":
            return self.stop()

        if not self.motion_done:
            return self.drive(self.straight_distance, self.straight_speed)

        self.motion_done = False
        self.pending_action = "NONE"
        self.mode = "FOLLOW"
        self.zebra_crossing = False
        self.intersection_lock = False
        self.get_logger().info("Straight completed")
        return self.stop()

    def handle_stop_mode(self):
        elapsed = (self.current_time - self.stop_start_time)
        if elapsed >= self.stop_sign_delay:
            # the PD keeps running (same as the original), output forced to 0
            return self.follow(0.0, 0.0)
        return self.follow(1.0, 1.0)

    def handle_special_mode(self):
        # stop while the light is red
        if self.tl_state == "RED":
            return self.stop()

        command = self.stop()
        elapsed = (self.current_time - self.special_start_time)

        if self.special_behavior == "WORKERS":
            speed_multiplier = 0.5 if elapsed >= self.special_slow_after else 1.0
            command = self.follow(speed_multiplier, 0.7)

        elif self.special_behavior == "ROUND":
            speed_multiplier = 0.45 if elapsed >= self.special_slow_after else 1.0
            command = self.follow(speed_multiplier, 1.2)

        elif self.special_behavior == "GIVE_WAY":

            # before the intersection
            if not self.zebra_crossing:
                command = self.follow(0.5, 1.0)

            # reached the zebra crossing
            else:
                # store the time only once
                if not self.intersection_lock:
                    self.intersection_lock = True
                    self.stop_start_time = (self.current_time)

                command = self.stop()
                stop_elapsed = (self.current_time - self.stop_start_time)

                # wait
                if stop_elapsed >= self.give_way_wait:
                    self.special_behavior = "NONE"
                    self.mode = "STRAIGHT"
                    self.pending_action = "STRAIGHT"
                    self.zebra_crossing = False
                    self.intersection_lock = False
                    self.motion_done = False
                    self.get_logger().info("Give way completed")

        if (self.special_behavior != "GIVE_WAY" and elapsed >= self.special_duration):
            self.special_behavior = "NONE"
            self.mode = "FOLLOW"
            self.intersection_lock = False
            self.get_logger().info("Special behavior completed")

        return command

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def _fsm_loop(self):
        self.current_time = (self.get_clock().now().nanoseconds / 1e9)

        # FSM
        self.update_fsm()

        # MODES
        if self.mode == "TURN":
            command = self.handle_turn_mode()

        elif self.mode == "WAIT_INTERSECTION":
            command = self.handle_wait_intersection()

        elif self.mode == "STRAIGHT":
            command = self.handle_straight_mode()

        elif self.mode == "SPECIAL":
            command = self.handle_special_mode()

        elif self.mode == "STOPPED":
            command = self.handle_stop_mode()

        else:
            command = self.handle_follow_mode()

        self.pub_cmd.publish(command)

        # Log only when the traffic light state changes
        tl_log = {
            "RED": "🔴 Red light detected",
            "YELLOW": "🟡 Yellow light detected",
            "GREEN": "🟢 Green light detected",
        }.get(self.tl_state, "⚪ Looking for a traffic light")
        if tl_log != self.last_tl_log:
            self.get_logger().info(tl_log)
            self.last_tl_log = tl_log


def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
