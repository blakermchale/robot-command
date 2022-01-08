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

from robot_command_interfaces.action import FollowTf
from builtin_interfaces.msg import Duration

import functools
import numpy as np


class CommanderClient(Node):
    def __init__(self, executor: Executor, namespace=None):
        super().__init__("commander_client", namespace=namespace)
        self.namespace = self.get_namespace().split("/")[-1]

        # Commander actions
        self._cli_follow_tf = ActionClient(self, FollowTf, "follow_tf")

        # Internal states
        self._timeout_sec = 60.0
        self._waiting_for_gh = False

        # Goal handles
        self._goal_handles = {}

        self._executor = executor
        self._executor.add_node(self)

    def send_follow_tf(self, distance: float, height: float, target_frame: str, run_time: float):
        self.reset()
        if not self._cli_follow_tf.wait_for_server(timeout_sec=self._timeout_sec):
            self.get_logger().error("No action server available")
            return
        sec = np.floor(run_time)
        nanosec = (run_time - sec)*10**9
        goal = FollowTf.Goal(distance=distance, height=height, target_frame=target_frame, run_time=Duration(sec=int(sec), nanosec=int(nanosec)))
        self.get_logger().info("Sending goal to `follow_tf`")
        future = self._cli_follow_tf.send_goal_async(goal, feedback_callback=self._feedback_follow_tf)
        future.add_done_callback(functools.partial(self._action_response, "follow_tf"))
        return future

    #############
    ## Helpers ##
    #############
    def reset(self):
        self._goal_handles = {}
        self._waiting_for_gh = True

    #FIXME: if goals are called and cancelled to quickly the action gets into a state where its goal is still running but not cancelled since it did not receive the goal handle in time
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
