#!/usr/bin/env python3
'''
ControlCenter Client
=======================

ControlCenter client to ROS2 actions. Easy action calling.
'''
from typing import List

from rclpy.executors import Executor
from rclpy.action import ActionClient

from robot_command_interfaces.action import SweepSearch
from geometry_msgs.msg import Polygon, Point32

import numpy as np
from ros2_utils.cli import setup_send_action, NodeClient


class ControlCenterClient(NodeClient):
    def __init__(self, executor: Executor, namespace=None, log_feedback=True):
        super().__init__("control_center_client", executor, namespace=namespace)
        self.log_feedback = log_feedback
        # Actions
        self._cli_sweep_search = ActionClient(self, SweepSearch, "sweep_search")

    def send_sweep_search(self, poly_points: np.ndarray, names: List[str], alt: float):
        @setup_send_action(self, self._cli_sweep_search, self._feedback_sweep_search)
        def send_action():
            msg_poly = Polygon()
            for r in poly_points:
                msg_poly.points.append(Point32(x=r[0],y=r[1]))
            return SweepSearch.Goal(area=msg_poly, names=names, alt=alt)
        return send_action

    ########################
    ## Feedback Callbacks ##
    ########################
    def _feedback_sweep_search(self, feedback):
        self.feedback = feedback.feedback
        if self.log_feedback:
            msg = ", ".join([f"{kv.key}: {kv.value}" for kv in feedback.feedback.info])
            self.get_logger().info(f"`sweep_search` feedback: current vehicle waypoints - {msg}", throttle_duration_sec=2.0)
