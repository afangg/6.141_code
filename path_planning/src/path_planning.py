#!/usr/bin/env python2

import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseStamped, PoseArray, Point
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory
from skimage.morphology import dilation, square
import heapq
import tf.transformations

class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        self.current_pose = None
        self.goal_pose = None

        # The map data, in row-major order, starting with (0,0).  Occupancy
        # probabilities are in the range [0,100].  Unknown is -1.
        self.map_occupancy_prob = None
        self.obstacles = None

        # The origin of the map [m, m, rad].  This is the real-world pose of the
        # cell (0,0) in the map.
        self.map_origin_pose = None

        # Map width, height [cells]
        self.map_dimensions = (0, 0)

        # The map resolution [m/cell]
        self.map_res = None

        # MAKE SURE YOU UNCOMMENT THIS AND SET THE ODOM TOPIC RIGHT
        # self.odom_topic = rospy.get_param("/particle_filter/odom_topic", "/odom")

        self.trajectory = LineTrajectory("/planned_trajectory")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.odom_sub = rospy.Subscriber("/pf/pose/odom", Odometry, self.odom_cb)


    def map_cb(self, msg):
        self.map_origin_pose = msg.info.origin
        _, _, self.theta = tf.transformations.euler_from_quaternion((self.map_origin_pose.orientation.x,
                                                                     self.map_origin_pose.orientation.y,
                                                                     self.map_origin_pose.orientation.z,
                                                                     self.map_origin_pose.orientation.w))
        self.map_dimensions = (msg.info.width, msg.info.height)
        map = np.reshape(msg.data, (msg.info.height, msg.info.width))

        self.map_occupancy_prob = dilation(map, square(4))
        print(self.map_occupancy_prob[339][359])
        self.map_res = msg.info.resolution
        obstacles1 = np.where(self.map_occupancy_prob == 100)
        obstacles2 = np.where(self.map_occupancy_prob == -1)

        self.obstacles = set(zip(obstacles1[1], obstacles1[0])).union(set(zip(obstacles2[1], obstacles2[0])) )

    def odom_cb(self, msg):
        if self.current_pose is None:
            print("intialize current pose")
        #Has .position (Point) and .orientation (Quaternion) attributes
        self.current_pose = msg.pose.pose
        pixel_coord = self.map_to_pixel((self.current_pose.position.x, self.current_pose.position.y))
        self.current_pose.position.x = pixel_coord[0]
        self.current_pose.position.y = pixel_coord[1]


    def goal_cb(self, msg):
        print("intialize goal pose")
        #Has .position (Point) and .orientation (Quaternion) attributes
        self.goal_pose = msg.pose
        pixel_coord = self.map_to_pixel((self.goal_pose.position.x, self.goal_pose.position.y))
        self.goal_pose.position.x = pixel_coord[0]
        self.goal_pose.position.y = pixel_coord[1]

        while self.current_pose is None or self.goal_pose is None or self.obstacles is None:
            print('waiting for all the required info')
        self.plan_path((self.current_pose.position.x, self.current_pose.position.y),
                           (self.goal_pose.position.x, self.goal_pose.position.y))

    # def trace_path(self, node, path = []):
    #     point = Point()
    #     map_coord = self.pixel_to_map((node[0][0], node[0][1]))
    #     point.x = map_coord[0]
    #     point.y = map_coord[1]
    #     path.append(point)
    #     if node[1] != None:
    #         return self.trace_path(node[1], path)
    #     else:
    #         path.reverse()
    #         return path

    def trace_path(self, start, end, parent):
        path = []
        point, node = Point(), end
        map_coord = self.pixel_to_map((node[0],node[1]))
        point.x = map_coord[0]
        point.y = map_coord[1]
        path.append(point)
        while parent[node] != node:
            node = parent[node]
            point = Point()
            map_coord = self.pixel_to_map((node[0],node[1]))
            point.x = map_coord[0]
            point.y = map_coord[1]
            path.append(point)
        return path[-1::-1]

    def astar(self, start_position, end_position, obstacles):
        queue = []
        heapq.heappush(queue, (0, start_position))
        visited = set()
        parent = {start_position: start_position}  # for path reconstruction
        mins = {start_position: 0}

        while queue:
            _, node = heapq.heappop(queue)
            if node in visited:
                continue
            if node == end_position:
                return self.trace_path(start_position, end_position, parent)
            visited.add(node)

            for neighbor in self.adj_nodes(node):
                if neighbor in visited or neighbor in obstacles:
                    continue
                new_cost = mins[node] + self.euclidian_cost(node, neighbor)
                if neighbor not in mins or mins[neighbor] > new_cost:
                    mins[neighbor] = new_cost
                    priority = new_cost + self.euclidian_cost(neighbor, end_position)
                    heapq.heappush(queue, (priority, neighbor))
                    parent[neighbor] = node
        return []

    def dijkstra(self, start_position, end_position, obstacles):
        queue = []
        heapq.heappush(queue, (0, start_position))
        visited = set()
        parent = {start_position: start_position}  # for path reconstruction
        mins = {start_position: 0}

        while queue:
            _, node = heapq.heappop(queue)
            if node in visited:
                continue
            if node == end_position:
                return self.trace_path(start_position, end_position, parent)
            visited.add(node)

            for neighbor in self.adj_nodes(node):
                if neighbor in visited or neighbor in obstacles:
                    continue
                new_cost = mins[node] + self.euclidian_cost(node, neighbor)
                if neighbor not in mins or mins[neighbor] > new_cost:
                    mins[neighbor] = new_cost
                    heapq.heappush(queue, (new_cost, neighbor))
                    parent[neighbor] = node
        return []

    # def astar(self, start_position, end_position, obstacles):
    #     print("starting astar")
    #
    #     queue = []
    #     heapq.heappush(queue, (0, 0, (start_position, None)))
    #     visited = set()
    #     while queue:
    #         _, cost, path = heapq.heappop(queue)
    #         current = path[0]
    #         if current in visited:
    #            continue
    #         if current == end_position:
    #            return self.trace_path(path, [])
    #         visited.add(current)
    #
    #         for neighbor in self.adj_nodes(current):
    #            if neighbor in visited or neighbor in obstacles:
    #                continue
    #
    #            new_cost = cost + self.euclidian_cost(current, neighbor)
    #            priority = new_cost + self.euclidian_cost(neighbor, end_position)
    #            heapq.heappush(queue, (priority, new_cost, (neighbor, path)))
    #     return []

    def plan_path(self, start_position, end_position):
        print('planning path')
        if end_position in self.obstacles:
            print("End goal is occupied")
            return

        path = self.dijkstra(start_position, end_position, self.obstacles)
        if not path:
            print("trajectory not found")
        self.trajectory.clear()
        for point in path:
            self.trajectory.addPoint(point)

        self.traj_pub.publish(self.trajectory.toPoseArray())
        # visualize trajectory Markers
        self.trajectory.publish_viz()

    def euclidian_cost(self, position1, position2):
        #Idk how the dubin curve thing right now so here's a 2D Euclidan heuristic lol
        return ((position2[0] - position1[0]))**2+((position2[1] - position1[1]))**2

    # def dubin_cost(self, pose1, pose2):
    #     x0 = pose1.position.x
    #     y0 = pose1.position.y
    #     _, _, theta0 = tf.transformations.euler_from_quaternion((pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w))
    #
    #     x1 = pose2.position.x
    #     y1 = pose2.position.y
    #     _, _, theta1 = tf.transformations.euler_from_quaternion((pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w))
    #     q0 = (x0, y0, theta0)
    #     q1 = (x1, y1, theta1)
    #     turning_radius = 0.5
    #     step_size = 0.1
    #     path = dubins.shortest_path(q0, q1, turning_radius)
    #     configurations, _ = path.sample_many(step_size)

    def adj_nodes(self, point):
        #return nodes that are adjacent to point
        adj_nodes = []
        for i in range (-1, 2):
            for j in range(-1, 2):
                if 0 <= point[0] + i < self.map_dimensions[0] and 0 <= point[1] + j < self.map_dimensions[1]:
                    adj_nodes.append((point[0] + i, point[1] + j))
        return adj_nodes

    def map_to_pixel(self, c):
        #converts a pose to a pixel
        shift = np.array([[self.map_origin_pose.position.x],[self.map_origin_pose.position.y]])
        c_np = np.array([[c[0]],[c[1]]])
        res = self.map_res
        rotation = np.array([[math.cos(self.theta), math.sin(self.theta)],[-1*math.sin(self.theta), math.cos(self.theta)]])
        p = (np.linalg.inv(rotation).dot((c_np-shift)))/res
        p = p.flatten()
        return round(p[0]),round(p[1])
    def pixel_to_map(self, c):
        #converts a pose to a pixel
        shift = np.array([[self.map_origin_pose.position.x],[self.map_origin_pose.position.y]])
        c_np = np.array([[c[0]],[c[1]]])

        res = self.map_res
        rotation = np.array([[math.cos(self.theta), math.sin(self.theta)],[-1*math.sin(self.theta), math.cos(self.theta)]])
        m = res*rotation.dot(c_np)+shift
        m = m.flatten()
        return m[0], m[1]

if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
