#!/usr/bin/env python3
'''
Control Center Shell
======
Shell interface for calling ROS2 actions of ControlCenter.
'''
import rclpy
from cmd2 import Cmd2ArgumentParser, with_argparser
from ros2_utils.cli import complete_action_call, ClientShell
from .control_center_client import ControlCenterClient
from argparse import ArgumentParser
import argparse


def point_type(s):
    try:
        x, y = map(float, s.split(','))
        return [x, y]
    except:
        raise argparse.ArgumentTypeError("Must be x,y")

def strlist_type(s):
    try:
        return s.split(',')
    except:
        raise argparse.ArgumentTypeError("Must be list of strings. Ex: 'ab,cd'")


class ControlCenterShell(ClientShell):
    prompt = "> "
    intro = "Welcome to control center shell! Type ? to list commands"
    def __init__(self, name) -> None:
        super().__init__(name, ControlCenterClient, persistent_history_file='~/.robot_command/control_center.dat')

    _sweep_search_argparser = Cmd2ArgumentParser(description='Sends `sweep_search` action.')
    _sweep_search_argparser.add_argument('names', type=strlist_type, help='distance away from tf (m)')
    _sweep_search_argparser.add_argument('alt', type=float, help='height to maintain (m)')
    _sweep_search_argparser.add_argument('poly_points', type=point_type, help='vertices of polygon (m)', nargs="+")
    @with_argparser(_sweep_search_argparser)
    def do_sweep_search(self, opts):
        future = self.cli.send_sweep_search(opts.poly_points, opts.names, opts.alt)
        complete_action_call(self.executor, future)

    @property
    def cli(self) -> ControlCenterClient:
        return self.client


def main(args=None):
    rclpy.init(args=args)

    parser = ArgumentParser()
    parser.add_argument("-n", "--name", default='hq', help='initial vehicle name')
    args, _ = parser.parse_known_args()

    shell = ControlCenterShell(args.name)
    shell.cmdloop()


if __name__=="__main__":
    main()
