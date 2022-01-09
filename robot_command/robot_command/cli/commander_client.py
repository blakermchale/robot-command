#!/usr/bin/env python3
'''
Commander Client
=======================

Commander client to ROS2 actions. Easy action calling.
'''
from rclpy.executors import Executor
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future

from robot_command_interfaces.action import FollowTf, CircleTf
from builtin_interfaces.msg import Duration

import functools
import numpy as np


class CommanderClient(Node):
    def __init__(self, executor: Executor, namespace=None):
        super().__init__("commander_client", namespace=namespace)
        self.namespace = self.get_namespace().split("/")[-1]

        # Commander actions
        self._cli_follow_tf = ActionClient(self, FollowTf, "follow_tf")
        self._cli_circle_tf = ActionClient(self, CircleTf, "circle_tf")

        # Internal states
        self._timeout_sec = 60.0
        self._waiting_for_gh = False

        # Goal handles
        self._goal_handles = {}

        self._executor = executor
        self._executor.add_node(self)

    def send_follow_tf(self, distance: float, height: float, approach_angle: float, target_frame: str, run_time: float):
        # self.reset()
        # if not self._cli_follow_tf.wait_for_server(timeout_sec=self._timeout_sec):
        #     self.get_logger().error("No action server available")
        #     return
        # sec = np.floor(run_time)
        # nanosec = (run_time - sec)*10**9
        # goal = FollowTf.Goal(distance=distance, height=height, approach_angle=approach_angle, target_frame=target_frame, run_time=Duration(sec=int(sec), nanosec=int(nanosec)))
        # self.get_logger().info("Sending goal to `follow_tf`")
        # future = self._cli_follow_tf.send_goal_async(goal, feedback_callback=self._feedback_follow_tf)
        # future.add_done_callback(functools.partial(self._action_response, "follow_tf"))
        # return future
        @setup_send_action(self, self._cli_follow_tf, self._feedback_follow_tf)
        def send_action():
            sec = np.floor(run_time)
            nanosec = (run_time - sec)*10**9
            return FollowTf.Goal(distance=distance, height=height, approach_angle=approach_angle, target_frame=target_frame, run_time=Duration(sec=int(sec), nanosec=int(nanosec)))
        return send_action

    def send_circle_tf(self, radius: float, height: float, loops: float, target_frame: str, run_time: float):
        @setup_send_action(self, self._cli_circle_tf, self._feedback_circle_tf)
        def send_action():
            sec = np.floor(run_time)
            nanosec = (run_time - sec)*10**9
            return CircleTf.Goal(radius=radius, height=height, loops=loops, target_frame=target_frame, run_time=Duration(sec=int(sec), nanosec=int(nanosec)))
        return send_action

    #############
    ## Helpers ##
    #############
    def reset(self):
        self._goal_handles = {}
        self._waiting_for_gh = True

    def _action_response(self, action_name: str, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Goal rejected for '{action_name}'")
            self._waiting_for_gh = False
            return
        self.get_logger().info(f"Goal accepted for '{action_name}'")
        self._goal_handles[action_name] = goal_handle

    ########################
    ## Feedback Callbacks ##
    ########################
    def _feedback_follow_tf(self, feedback):
        self.get_logger().info(f"`follow_tf` feedback: running for {feedback.feedback.time.sec}s", throttle_duration_sec=2.0)

    def _feedback_circle_tf(self, feedback):
        self.get_logger().info(f"`circle_tf` feedback: running for {feedback.feedback.time.sec}s", throttle_duration_sec=2.0)


def setup_send_action(self, action_cli, feedback_cb):
    def inner(func):
        self.reset()
        if not action_cli.wait_for_server(timeout_sec=self._timeout_sec):
            self.get_logger().error(f"No action server available for `{action_cli._action_name}`")
            return
        goal = func()
        self.get_logger().info(f"Sending goal to `{action_cli._action_name}`")
        future = action_cli.send_goal_async(goal, feedback_callback=feedback_cb)
        future.add_done_callback(functools.partial(self._action_response, action_cli._action_name))
        return future
    return inner
