#!/usr/bin/env python3
'''
Commander Client
=======================

Commander client to ROS2 actions. Easy action calling.
'''
from rclpy.executors import Executor
from rclpy.action import ActionClient

from robot_command_interfaces.action import FollowTf, CircleTf
from builtin_interfaces.msg import Duration

import functools
import numpy as np
from robot_control.cli.common import setup_send_action, NodeClient


class CommanderClient(NodeClient):
    def __init__(self, executor: Executor, namespace=None):
        super().__init__("commander_client", executor, namespace=namespace)
        # Commander actions
        self._cli_follow_tf = ActionClient(self, FollowTf, "follow_tf")
        self._cli_circle_tf = ActionClient(self, CircleTf, "circle_tf")

    def send_follow_tf(self, distance: float, height: float, approach_angle: float, target_frame: str, run_time: float):
        @setup_send_action(self, self._cli_follow_tf, self._feedback_follow_tf)
        def send_action():
            sec = np.floor(run_time)
            nanosec = (run_time - sec)*10**9
            return FollowTf.Goal(distance=distance, height=height, approach_angle=approach_angle, target_frame=target_frame, run_time=Duration(sec=int(sec), nanosec=int(nanosec)))
        return send_action

    def send_circle_tf(self, radius: float, height: float, speed: float, target_frame: str, run_time: float):
        @setup_send_action(self, self._cli_circle_tf, self._feedback_circle_tf)
        def send_action():
            sec = np.floor(run_time)
            nanosec = (run_time - sec)*10**9
            return CircleTf.Goal(radius=radius, height=height, speed=speed, target_frame=target_frame, run_time=Duration(sec=int(sec), nanosec=int(nanosec)))
        return send_action

    ########################
    ## Feedback Callbacks ##
    ########################
    def _feedback_follow_tf(self, feedback):
        self.get_logger().info(f"`follow_tf` feedback: running for {feedback.feedback.time.sec}s", throttle_duration_sec=2.0)

    def _feedback_circle_tf(self, feedback):
        self.get_logger().info(f"`circle_tf` feedback: running for {feedback.feedback.time.sec}s", throttle_duration_sec=2.0)
