#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.client import GoalStatus
from rclpy.action.server import ServerGoalHandle
# Interfaces
from geometry_msgs.msg import PolygonStamped, Polygon, Point32, PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Path
from robot_control_interfaces.action import FollowWaypoints
from robot_command_interfaces.action import SweepSearch
from diagnostic_msgs.msg import KeyValue
# General
from shapely.geometry import Polygon as SPolygon
import numpy as np
from .split_polygon import split_polygon_voronoi
from .polygon_path import sweep_polygon
from ros2_utils import NpVector4, NpPose
import functools
from ros2_utils.ros import convert_axes_from_msg, AxesFrame
from robot_control.cli.drone_client import DroneClient, create_drone_client
from ros2_utils.cli import gh_state_machine, CompleteActionState


def convert_shapely_to_msg(polygon: SPolygon, alt=1.0) -> Polygon:
    msg = PolygonStamped()
    msg_poly = Polygon()
    for x,y in np.asfarray(polygon.exterior.xy).T:
        pnt = Point32(x=x,y=y,z=alt)
        msg_poly.points.append(pnt)
    msg.polygon = msg_poly
    return msg


def convert_msg_to_shapely(msg: Polygon) -> SPolygon:
    points = []
    for p in msg.points:
        points.append([p.x, p.y])
    return SPolygon(points)


class ControlCenter(Node):
    def __init__(self):
        super().__init__("control_center")
        # Setup loop
        timer_period = 1/20  # seconds
        self._timer_update = self.create_timer(timer_period, self.update)
        self._check_rate = self.create_rate(20)  # Rate to check conditions in actions
        # Publishers
        self._pub_search_area = self.create_publisher(PolygonStamped, "search/area", 1)
        self._search_area = SPolygon([[10, 10], [40, 0], [60,20], [20, 50], [0, 30], [10, 10]])
        # Actions
        # TODO: add action for receiving polygon and list of vehicle names, then distributing paths to those vehicles and searching, closest vehicle to split region should go there
        self._cli_sweep_search = ActionServer(self, SweepSearch, "sweep_search", self._handle_sweep_search_goal, cancel_callback=self._handle_sweep_search_cancel)

    def update(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        poly_msg = convert_shapely_to_msg(self._search_area,alt=-3.0)
        poly_msg.header = header
        poly_msg.polygon = convert_axes_from_msg(poly_msg.polygon, AxesFrame.URHAND, AxesFrame.RHAND)
        self._pub_search_area.publish(poly_msg)

    async def _handle_sweep_search_goal(self, goal : ServerGoalHandle):
        result = SweepSearch.Result()
        req : SweepSearch.Goal = goal.request
        # req.area.points = [Point32(x=0.0,y=0.0,z=0.0), Point32(x=30.0,y=-10.0,z=0.0),  Point32(x=23.0,y=27.0,z=0.0),  Point32(x=0.0,y=10.0,z=0.0),  Point32(x=0.0,y=0.0,z=0.0)]
        # req.names = ["drone_0", "drone_1", "drone_2"]
        # req.alt = -3.0
        if not req.names or not req.area.points:
            self.get_logger().error(f"SweepSearch: not enough points or vehicles to distribute")
            goal.abort()
            return SweepSearch.Result()
        names = req.names
        alt = req.alt
        poly = convert_msg_to_shapely(req.area)
        self._search_area = poly
        data = {}
        # Split area into paths
        num_vehicles = len(names)
        regions = split_polygon_voronoi(poly, num_vehicles)
        self.get_logger().info(f"SweepSearch: split area with voronoi")
        paths = []
        for i, r in enumerate(regions.geoms):
            paths.append(sweep_polygon(r))
            self.get_logger().info(f"SweepSearch: generated sweep for region {i}")
        # Connecting clients
        self.get_logger().info(f"SweepSearch: Connecting to clients...")
        clients = {}
        executor = SingleThreadedExecutor()
        for n in names:
            data[n] = {"future": None, "state": CompleteActionState.END}
            clients[n] = DroneClient(executor, namespace=n, log_feedback=False)
            self.get_logger().info(f"SweepSearch: connected to \"{n}\"")
        # Distribute to vehicles and instantiate connections to action API
        self.get_logger().info(f"SweepSearch: distributing paths to vehicles")
        for n in names:
            path = paths.pop()
            shape = (path.shape[0],1)
            xyz_yaw_frame = np.hstack((path, alt*np.ones(shape), np.zeros(shape), 1*np.ones(shape)))
            data[n] = {}
            drone_client : DroneClient = clients[n]
            data[n]["future"] = drone_client.send_follow_waypoints(xyz_yaw_frame, 1.0)
            data[n]["state"] = CompleteActionState.WAIT_GH
            self.get_logger().info(f"SweepSearch: distributed path to vehicle \"{n}\"")
        # Wait for results
        self.get_logger().info(f"SweepSearch: waiting on goal handle")
        wait_names = names.copy()
        group_state = CompleteActionState.WAIT_GH
        while True:
            feedback = SweepSearch.Feedback()
            for n in names:
                if clients[n].feedback: feedback.info.append(KeyValue(key=n, value=str(clients[n].feedback.idx)))
            goal.publish_feedback(feedback)
            # Can only cancel when goal handle result has not been retrieved
            if goal.is_cancel_requested and group_state in [CompleteActionState.WAIT_GH, CompleteActionState.WAIT_GH_FUTURE]:  # Can't cancel until goal handles are received
                for n in names:
                    if "cancel_future" not in data[n]: data[n]["cancel_future"] = data[n]["goal_handle"].cancel_goal_async()
                cancel_response = True
                for n in names:
                    if not data[n]["cancel_future"].done(): cancel_response = False
                if not cancel_response:
                    for n in names:
                        clients[n].executor.spin_once()
                    continue  # skip stepping forward in stages until cancel gets a response
            # Consider it canceled when all action clients have returned a result
            if goal.is_cancel_requested and group_state == CompleteActionState.CHECK_STATUS:
                group_state = CompleteActionState.CANCELED
                self.get_logger().info("SweepSearch: Cancelled!")
                goal.canceled()
                return result
            # Step goal handles through stages until a result is found
            for n in wait_names:
                clients[n].executor.spin_once()
                data[n]["state"] = gh_state_machine(data[n])
            if wait_names:
                for n in wait_names:
                    if data[n]["state"] != group_state:  # Remove once its moved to the next stage
                        wait_names.remove(n)
            else:  # Reset when all states have moved on to the next stage
                if list(data.values())[0]["state"] in [CompleteActionState.SUCCEEDED, CompleteActionState.CANCELED, CompleteActionState.FAILURE]:
                    for n in names:
                        if data[n]["state"] == CompleteActionState.FAILURE:
                            self.get_logger().error(f"SweepSearch: {n} failed...")
                            goal.abort()
                            return SweepSearch.Result()
                    break  # assume it was a success
                self.get_logger().info(f"SweepSearch: moving to next stage {data[n]['state'].name}")
                if group_state+1 < CompleteActionState.END: wait_names = names.copy()
                group_state = CompleteActionState(group_state+1)
        goal.succeed()
        self.get_logger().info("SweepSearch: searched area!")
        return result

    def _handle_sweep_search_cancel(self, cancel):
        return CancelResponse.ACCEPT

    def perform_search(self, names, alt):
        data = {}
        for n in names:
            def sub_pose(msg: PoseStamped, name: str):
                data[name]["pose"] = NpPose.from_ros(msg.pose)
            def sub_path(msg: Path, name: str):
                data[name]["path"] = msg
            data[n] = {
                "pub": {
                    "path": self.create_publisher(Path, f"/{n}/cmd/path/ned", 1)
                },
                "sub": {
                    "pose": self.create_subscription(PoseStamped, f"/{n}/pose", functools.partial(sub_pose, name=n), 1),
                    "path": self.create_subscription(Path, f"/{n}/path", functools.partial(sub_path, name=n), 1),
                }
            }
        num_vehicles = len(names)
        poly = self._search_area
        regions = split_polygon_voronoi(poly, num_vehicles)
        self.get_logger().info(f"Search: split area with voronoi")
        paths = []
        for i, r in enumerate(regions.geoms):
            paths.append(sweep_polygon(r))
            self.get_logger().info(f"Search: generated sweep for region {i}")
        for name in data.keys():
            path = paths.pop()
            msg = Path()
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "map"
            for (x, y) in path:
                p = PoseStamped()
                p.pose.position.x = x
                p.pose.position.y = y
                p.pose.position.z = alt
                p.header = header
                msg.poses.append(p)
            msg.header = header
            data[name]["pub"]["path"].publish(msg)
        self.get_logger().info("Search: sent all path commands")
        while True:
            if data[n].get("path") and not data[n]["path"].paths:
                break
        self.get_logger().info("Search: vehicles finished paths")


def main(args=None):
    rclpy.init(args=args)
    cc = ControlCenter()
    executor = MultiThreadedExecutor()
    # cc.perform_search(["drone_0", "drone_1", "drone_2"], -3.0)
    rclpy.spin(cc, executor=executor)


if __name__=="__main__":
    main()
