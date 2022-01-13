#!/usr/bin/env python3
'''
tf_processor.py
---------------

Takes TF frames and processes them in some way. Common processing methods are averaging, kalman filters, and raw. 
Constantly publishes these at the current time.
'''
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from rclpy.action import ActionServer
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
# ROS interfaces
from robot_command_interfaces.action import CircleTf, FollowTf
from geometry_msgs.msg import Twist, Pose, TransformStamped
from std_msgs.msg import Header
from argparse import ArgumentParser
import argparse
from enum import IntEnum, auto


class MethodType(IntEnum):
    RAW=0
    AVERAGE=auto()
# TODO: add service for adding and removing processors, allow for list to be passed with target frame and method
class TfProcessor(Node):
    def __init__(self):
        super().__init__("tf_processor")
        self._namespace = self.get_namespace().split("/")[1]

        # Main loop
        timer_period = 1/20  # seconds
        self._timer_update = self.create_timer(timer_period, self.update)

        # Internal vars
        self._source_frame = "map"

        # Map for maintaining current tf's being processed and the method for processing
        self._processors = {}

        # TF
        # self._tf_node = rclpy.create_node('commander_tf', use_global_arguments=False)  # needed to access /tf to get around the namespace
        self._tfbuff = Buffer()
        self._tfl = TransformListener(buffer=self._tfbuff, node=self)
        self._br = TransformBroadcaster(self)
        self.get_logger().info(f"---------Initialized {self._namespace} TF processor ----------")

    def update(self):
        if not self._processors.keys():  # Exit early if no processors
            self.get_logger().warn("No processors at start", once=True)
            return
        tf_list = []
        header = Header()
        header.frame_id = self._source_frame
        header.stamp = self.get_clock().now().to_msg()
        for k, v in self._processors.items():
            try:
                tf = self._tfbuff.lookup_transform(self._source_frame, k, Time())
                method = self._processors[k]["method"]
                if method == MethodType.RAW:
                    self._processors[k]["value"] = tf.transform
                # elif method == "average":
                #     self._processors[k]["info"]["count"] += 1
                else:
                    self.get_logger().error(f"Method {method.name.lower()} not implemented yet")
                    continue
            except Exception as e: #possibly tf2.LookupException or tf2.ExtrapolationException
                # self.get_logger().error(f"{str(e)}", throttle_duration_sec=1.0)
                if self._processors[k]["value"] is None:
                    self.get_logger().warn(f"Haven't received value for {k} yet", throttle_duration_sec=5.0)
                continue
            tf = TransformStamped()
            tf.header = header
            tf.transform = self._processors[k]["value"]
            tf.child_frame_id = v["output_frame"]
            tf_list.append(tf)
        if tf_list:  # only publish when tf list has been filled
            self._br.sendTransform(tf_list)

    def add_processor(self, target_frame: str, method: MethodType):
        """Adds processor to map if the target frame is not already being processed. Returns flag indicating success."""
        method_name = method.name.lower()
        if not self._processors.get(target_frame):
            output_frame = f"{target_frame}/{method_name}"
            info = {}  # variables specific to the method type
            if method == MethodType.RAW:
                pass
            elif method == MethodType.AVERAGE:  # needs to track total number of tfs received to average evenly
                info = {"count": 0}
            else:
                self.get_logger().error(f"Method {method_name} not implemented yet")
                return False
            self._processors[target_frame] = {"output_frame": output_frame, "method": method, "value": None, "info": info}
        else:
            return False
        self.get_logger().info(f"Added processor for target {target_frame} with method {method_name}")
        return True

    def remove_processor(self, target_frame):
        """Removes processor from map if the target frame is already being processed. Returns flag indicating success."""
        if self._processors.get(target_frame):
            self._processors.pop(target_frame)
        else:
            return False
        return True


def pair(s):
    try:
        tf_frame, method = map(str, s.split(','))
        method = MethodType[method.upper()]
        return (tf_frame, method)
    except:
        raise argparse.ArgumentTypeError("Pair must be tf frame and method for processor.")


def main(args=None):
    rclpy.init(args=args)
    parser = ArgumentParser()
    parser.add_argument('--tf-pair', help="Pair of tf frame and method for processing.", type=pair, nargs="+")
    args, _ = parser.parse_known_args()
    tf_processor = TfProcessor()
    if args.tf_pair:
        processor_pairs = args.tf_pair
        for p in processor_pairs:
            tf_processor.add_processor(p[0], p[1])
    executor = MultiThreadedExecutor()
    rclpy.spin(tf_processor, executor)


if __name__=="__main__":
    main()
