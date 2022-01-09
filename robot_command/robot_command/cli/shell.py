#!/usr/bin/env python3
'''
Shell
======
Shell interface for calling ROS2 actions.
'''
import rclpy
from cmd2 import Cmd, Cmd2ArgumentParser, with_argparser
from rclpy.executors import MultiThreadedExecutor
from robot_control.cli.common import complete_action_call, check_futures_done
from .commander_client import CommanderClient
from argparse import ArgumentParser


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

# https://pymotw.com/2/cmd/
# https://pypi.org/project/cmd2/
class CommanderShell(Cmd):
    prompt = "> "
    intro = "Welcome to commander shell! Type ? to list commands"
    def __init__(self, name) -> None:
        super().__init__(persistent_history_file='~/.robot_command/cmd2_history.dat')
        self.executor = MultiThreadedExecutor()
        client = CommanderClient(self.executor, namespace=name)
        self.clients_archive = {name: client}
        self.client = client
        self.name = name

    def sigint_handler(self, signum: int, _) -> None:
        cancel_futures = []
        if self.client._waiting_for_gh:
            print("Cannot cancel until goal is retrieved")
            return
        for k, v in list(self.client._goal_handles.items()):
            print(f"\nCancelling `{k}`!")
            cancel_futures.append(v.cancel_goal_async())
            self.client._goal_handles.pop(k)
        while self.executor._context.ok() and not check_futures_done(cancel_futures) and not self.executor._is_shutdown:
            print("waiting for cancels to complete")
            self.executor.spin_once()
        print("Finished ^C")
        super().sigint_handler(signum, _)

    _set_name_argparser = Cmd2ArgumentParser(description='Changes client to new vehicle name.')
    _set_name_argparser.add_argument('name', type=str, help='vehicle namespace')
    @with_argparser(_set_name_argparser)
    def do_set_name(self, opts):
        if opts.name not in self.clients_archive.keys():
            self.executor = MultiThreadedExecutor()
            self.clients_archive[opts.name] = CommanderClient(self.executor, namespace=opts.name)
        self.client = self.clients_archive[opts.name]
        self.name = opts.name

    def do_get_name(self, opts):
        """Gets current client vehicle name."""
        print(f"Vehicle name is '{self.client.namespace}'")

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

    def do_exit(self, args):
        """Exit shell."""
        print("Exiting")
        return True

    def default(self, inp):
        if inp in ["x", "q"]:
            return self.do_exit(inp)
        print("Default not implemented: {}".format(inp))

    do_EOF = do_exit


def main(args=None):
    rclpy.init(args=args)

    parser = ArgumentParser()
    parser.add_argument("-n", "--name", default='drone_0', help='initial vehicle name')
    args, _ = parser.parse_known_args()

    shell = CommanderShell(args.name)
    shell.cmdloop()


if __name__=="__main__":
    main()
