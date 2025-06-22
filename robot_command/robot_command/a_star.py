import numpy as np
from dataclasses import dataclass
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, Header
from rclpy.node import Node
from rclpy.action import ActionServer
from queue import PriorityQueue


"""
1. Pull in Occupancy Grid message
2. Convert occupancy grid to numpy array
3. Parse offset of occupancy grid in the world from metadata
4. Run A* algorithm on the grid
5. Return path as a Path message

Inspiration:
https://www.redblobgames.com/pathfinding/a-star/implementation
https://en.wikipedia.org/wiki/A*_search_algorithm
"""


class AStarNode(Node):

    def __init__(self):
        self.map_sub = self.create_subscription("map", OccupancyGrid, self.map_callback, 1)
        self.default_height = 0.0
        pass

    def map_callback(self, msg: OccupancyGrid):
        self.grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    def initialize(self, start, goal):
        self.start : np.ndarray = np.asfarray(start)
        self.goal : np.ndarray = np.asfarray(goal)
        self.grid : np.ndarray = np.asfarray(grid)

    def distance(self, position, destination):
        d = np.linalg.norm(position - destination)
        return d
    
    def heuristic(self, position, destination):
        # Using Euclidean distance as heuristic
        return self.distance(position, destination)
    
    def neighbors(self, position):
        x, y = position
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.grid.shape[0] and 0 <= ny < self.grid.shape[1]:
                    if self.grid[nx, ny] == 0:
                        neighbors.append(np.array([nx, ny]))
        return neighbors
    
    def run(self):
        open_set = PriorityQueue()
        open_set.put(self.start, 0)
        parent = {}
        # Cost from beginning to the current node
        cost_tracked = {}
        parent[self.start] = None
        cost_tracked[self.start] = 0
        
        while not open_set.empty():
            current = open_set.get()

            # Found the goal!
            if np.array_equal(current, self.goal):
                break

            for next in self.neighbors(current):
                new_cost = cost_tracked[current] + self.heuristic(current, next)
                if next not in cost_tracked or new_cost < cost_tracked[next]:
                    cost_tracked[next] = new_cost
                    priority = new_cost + self.heuristic(next, self.goal)
                    open_set.put(next, priority)
                    parent[next] = current
        
        # Reconstruct path
        path = []
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = []
        current = self.goal
        while current is not None:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = current[0]
            pose.pose.position.y = current[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            path.poses.append(pose)
            current = parent.get(current, None)
        
        path.reverse()


def main():
    pass


if __name__ == "__main__":
    main()
