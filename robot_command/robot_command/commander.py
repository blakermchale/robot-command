#!/usr/bin/env python3
import rclpy
from rclpy import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from rclpy.action import ActionServer
from tf2_ros import TransformListener, Buffer
# ROS interfaces
from robot_command_interfaces.action import CircleTf, FollowTf
from geometry_msgs.msg import Twist, Pose


class Commander(Node):
    def __init__(self):
        super().__init__("commander")
        self._namespace = self.get_namespace().split("/")[1]
        # Publishers
        self._pub_cmd_vel = self.create_publisher(Twist, "cmd/velocity", 1)
        self._pub_cmd_ned = self.create_publisher(Pose, "cmd/ned", 1)
        self._pub_cmd_frd = self.create_publisher(Pose, "cmd/frd", 1)
        # Actions
        self._cli_follow_tf = ActionServer(self, FollowTf, "follow_tf", self._handle_follow_tf_goal)
        self._cli_circle_tf = ActionServer(self, CircleTf, "circle_tf", self._handle_circle_tf_goal)
        # TF
        # self._tf_node = rclpy.create_node('commander_tf', use_global_arguments=False)  # needed to access /tf to get around the namespace
        self._tfbuff = Buffer()
        self._tfl = TransformListener(buffer=self._tfbuff, node=self)

    #############
    ## Actions ##
    #############
    def _handle_follow_tf_goal(self, goal):
        def get_request(goal) -> FollowTf.Goal:
            return goal.request
        req = get_request(goal)
        while rclpy.ok():
            try:
                tf = self._tfbuff.lookup_transform(self._namespace, req.target_frame, Time())
            except Exception as e: #possibly tf2.LookupException or tf2.ExtrapolationException
                self.get_logger().error(f"{str(e)}", throttle_duration_sec=1.0)
                continue

    def _handle_circle_tf_goal(self, goal):
        def get_request(goal) -> CircleTf.Goal:
            return goal.request
        req = get_request(goal)
        approach_ang = 0.0
        while rclpy.ok():
            try:
                tf = self._tfbuff.lookup_transform(self._namespace, req.target_frame, Time())
            except Exception as e: #possibly tf2.LookupException or tf2.ExtrapolationException
                self.get_logger().error(f"{str(e)}", throttle_duration_sec=1.0)
                continue


def main(args=None):
    rclpy.init(args=args)
    commander = Commander()
    executor = MultiThreadedExecutor()
    rclpy.spin(commander, executor)


if __name__=="__main__":
    main()
