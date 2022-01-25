#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
# Interfaces
from geometry_msgs.msg import PolygonStamped, Polygon, Point32, PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Path
# General
from shapely.geometry import Polygon as SPolygon
import numpy as np
from .split_polygon import split_polygon_voronoi
from .polygon_path import sweep_polygon
from ros2_utils import NpVector4, NpPose
import functools
from ros2_utils.ros import convert_axes_from_msg, AxesFrame


def convert_shapely_to_msg(polygon: SPolygon, alt=1.0):
    msg = PolygonStamped()
    msg_poly = Polygon()
    for x,y in np.asfarray(polygon.exterior.xy).T:
        pnt = Point32(x=x,y=y,z=alt)
        msg_poly.points.append(pnt)
    msg.polygon = msg_poly
    return msg


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
        # self._cli_search = ActionClient(self, )

    def update(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        poly_msg = convert_shapely_to_msg(self._search_area,alt=-3.0)
        poly_msg.header = header
        poly_msg.polygon = convert_axes_from_msg(poly_msg.polygon, AxesFrame.URHAND, AxesFrame.RHAND)
        self._pub_search_area.publish(poly_msg)

    def perform_search(self):
        alt = -3.0
        names = ["drone_0", "drone_1", "drone_2"]
        data = {}
        for n in names:
            def sub_pose(msg: PoseStamped, name: str):
                data[n]["pose"] = NpPose.ros(msg.pose)
            data[n] = {
                "pub": {
                    "path": self.create_publisher(Path, f"/{n}/cmd/path/ned", 1)
                },
                "sub": {
                    "pose": self.create_subscription(PoseStamped, f"/{n}/pose", functools.partial(sub_pose, name=n), 1)
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


def main(args=None):
    rclpy.init(args=args)
    cc = ControlCenter()
    executor = MultiThreadedExecutor()
    cc.perform_search()
    rclpy.spin(cc, executor=executor)


if __name__=="__main__":
    main()
