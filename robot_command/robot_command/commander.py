#!/usr/bin/env python3
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from rclpy.action import ActionServer, CancelResponse
from tf2_ros import TransformListener, Buffer
# ROS interfaces
from robot_command_interfaces.action import CircleTf, FollowTf
from geometry_msgs.msg import Twist, Pose
# General
import numpy as np
from ros2_utils import AxesFrame, convert_axes_from_msg, wrap_to_pi, NpVector4


class Commander(Node):
    def __init__(self):
        super().__init__("commander")
        self._namespace = self.get_namespace().split("/")[1]
        # Publishers
        self._pub_cmd_vel = self.create_publisher(Twist, "cmd/velocity", 1)
        self._pub_cmd_ned = self.create_publisher(Pose, "cmd/ned", 1)
        self._pub_cmd_frd = self.create_publisher(Pose, "cmd/frd", 1)
        # Actions
        self._cli_follow_tf = ActionServer(self, FollowTf, "follow_tf", self._handle_follow_tf_goal, cancel_callback=_handle_accept_action_cancel)
        self._cli_circle_tf = ActionServer(self, CircleTf, "circle_tf", self._handle_circle_tf_goal, cancel_callback=_handle_accept_action_cancel)
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
        self.get_logger().debug(f"FollowTf: following {req.target_frame} with distance {req.distance}m, height {req.height}m, and angle {req.approach_angle}deg for {req.run_time}s")
        start_time = self.get_clock().now()
        run_time = Duration.from_msg(req.run_time)
        fail_time = Duration(seconds=10.0)
        start_fail_time = start_time

        while rclpy.ok():
            feedback = FollowTf.Feedback()
            feedback.time = (self.get_clock().now() - start_time).to_msg()
            goal.publish_feedback(feedback)
            if goal.is_cancel_requested:
                self.get_logger().info("FollowTf: Cancelled!")
                goal.canceled()
                return FollowTf.Result()
            try:
                tf = self._tfbuff.lookup_transform("map", req.target_frame, Time())
                # upward_angle = np.arcsin(req.height, req.distance)  # Useful if camera has a gimbal and can angle towards the object
                lateral = req.distance
                yaw = NpVector4.from_ros(tf.transform.rotation).yaw
                approach_angle = wrap_to_pi(np.deg2rad(req.approach_angle) + yaw)
                pose = Pose()
                pose.position.x = tf.transform.translation.x - np.cos(approach_angle)*lateral
                pose.position.y = tf.transform.translation.y - np.sin(approach_angle)*lateral
                pose.position.z = tf.transform.translation.z + req.height
                pose.orientation = NpVector4.from_rpy(0.0, 0.0, approach_angle).get_quat_msg()
                pose = convert_axes_from_msg(pose,AxesFrame.RHAND,AxesFrame.URHAND)
                self._pub_cmd_ned.publish(pose)
                # TODO: maybe use velocity commands and PID that focuses on target position, similar to ignition
                # from robot_control.utils.pid_position_controller import PIDPositionController
            except Exception as e: #possibly tf2.LookupException or tf2.ExtrapolationException
                self.get_logger().error(f"{str(e)}", throttle_duration_sec=1.0)
                if self.get_clock().now() - start_fail_time > fail_time:  # exit if error has occurred for fail time length
                    self.get_logger().error(f"Couldn't get transform consistently for {fail_time}")
                    goal.abort()
                    return FollowTf.Result()
                continue
            start_fail_time = self.get_clock().now()  # reset fail time
            if self.get_clock().now() - start_time > run_time:
                self.get_logger().info("Followed tf for full run time")
                goal.succeed()
                return FollowTf.Result()

    def _handle_circle_tf_goal(self, goal):
        """Circle around TF for given time and number of loops at a radius and height away."""
        def get_request(goal) -> CircleTf.Goal:
            return goal.request
        req = get_request(goal)
        self.get_logger().debug(f"CircleTf: circling {req.target_frame} with radius {req.radius}m, height {req.height}m, and speed {req.speed} deg/s for {req.run_time}s")
        run_time = Duration.from_msg(req.run_time)
        speed = np.deg2rad(req.speed)

        start_time = self.get_clock().now()
        fail_time = Duration(seconds=10.0)
        start_fail_time = start_time

        while rclpy.ok():
            dur_dt = self.get_clock().now() - start_time
            dt = dur_dt.nanoseconds * 10**-9
            angle = wrap_to_pi(speed * dt)

            feedback = CircleTf.Feedback()
            feedback.time = dur_dt.to_msg()
            goal.publish_feedback(feedback)

            if goal.is_cancel_requested:
                self.get_logger().info("CircleTf: Cancelled!")
                goal.canceled()
                return CircleTf.Result()

            try:
                tf = self._tfbuff.lookup_transform("map", req.target_frame, Time())
                pose = Pose()
                pose.position.x = tf.transform.translation.x - np.cos(angle)*req.radius
                pose.position.y = tf.transform.translation.y - np.sin(angle)*req.radius
                pose.position.z = tf.transform.translation.z + req.height
                pose.orientation = NpVector4.from_rpy(0.0, 0.0, angle).get_quat_msg()
                pose = convert_axes_from_msg(pose,AxesFrame.RHAND,AxesFrame.URHAND)
                self._pub_cmd_ned.publish(pose)
            except Exception as e: #possibly tf2.LookupException or tf2.ExtrapolationException
                self.get_logger().error(f"{str(e)}", throttle_duration_sec=1.0)
                if self.get_clock().now() - start_fail_time > fail_time:  # exit if error has occurred for fail time length
                    self.get_logger().error(f"Couldn't get transform consistently for {fail_time}")
                    goal.abort()
                    return CircleTf.Result()
                continue
            start_fail_time = self.get_clock().now()  # reset fail time
            if self.get_clock().now() - start_time > run_time:
                self.get_logger().info("Circled tf for full run time")
                goal.succeed()
                return CircleTf.Result()


def _handle_accept_action_cancel(cancel):
    """Generic callback for accepting cancel."""
    return CancelResponse.ACCEPT


def main(args=None):
    rclpy.init(args=args)
    commander = Commander()
    executor = MultiThreadedExecutor()
    rclpy.spin(commander, executor)


if __name__=="__main__":
    main()
