#!/usr/bin/env python3
'''
Shell
======
Shell interface for calling ROS2 actions.
'''
import rclpy
from cmd2 import Cmd2ArgumentParser, with_argparser
from robot_control.cli.common import complete_action_call, ClientShell
from .commander_client import CommanderClient
from argparse import ArgumentParser


class CommanderShell(ClientShell):
    prompt = "> "
    intro = "Welcome to commander shell! Type ? to list commands"
    def __init__(self, name) -> None:
        super().__init__(name, CommanderClient, persistent_history_file='~/.robot_command/commander.dat')

    _follow_tf_argparser = Cmd2ArgumentParser(description='Sends `follow_tf` action.')
    _follow_tf_argparser.add_argument('distance', type=float, help='distance away from tf (m)')
    _follow_tf_argparser.add_argument('height', type=float, help='height to maintain (m)')
    _follow_tf_argparser.add_argument('approach_angle', type=float, help='angle to look at tf from in NED (deg)')
    _follow_tf_argparser.add_argument('target_frame', type=str, help='tf target frame')
    _follow_tf_argparser.add_argument('run_time', type=float, help='time to run for (s)')
    @with_argparser(_follow_tf_argparser)
    def do_follow_tf(self, opts):
        future = self.client.send_follow_tf(opts.distance, opts.height, opts.approach_angle, opts.target_frame, opts.run_time)
        complete_action_call(self.client, self.executor, future, "follow_tf")

    _circle_tf_argparser = Cmd2ArgumentParser(description='Sends `circle_tf` action.')
    _circle_tf_argparser.add_argument('radius', type=float, help='radius of circle (m)')
    _circle_tf_argparser.add_argument('height', type=float, help='height to maintain (m)')
    _circle_tf_argparser.add_argument('speed', type=float, help='speed to rotate around tf (deg/s)')
    _circle_tf_argparser.add_argument('target_frame', type=str, help='tf target frame')
    _circle_tf_argparser.add_argument('run_time', type=float, help='time to run for (s)')
    @with_argparser(_circle_tf_argparser)
    def do_circle_tf(self, opts):
        future = self.client.send_circle_tf(opts.radius, opts.height, opts.speed, opts.target_frame, opts.run_time)
        complete_action_call(self.client, self.executor, future, "circle_tf")


def main(args=None):
    rclpy.init(args=args)

    parser = ArgumentParser()
    parser.add_argument("-n", "--name", default='drone_0', help='initial vehicle name')
    args, _ = parser.parse_known_args()

    shell = CommanderShell(args.name)
    shell.cmdloop()


if __name__=="__main__":
    main()
