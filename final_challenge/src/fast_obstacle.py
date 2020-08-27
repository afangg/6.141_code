#!/usr/bin/env python2

import rospy
import numpy as np
import time
from threading import Lock

from threading import Lock
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseArray, Point
from nav_msgs.msg import Odometry
import tf

class FastObstacle(object):
    """ For fast obstacle avoidance.
    """
    K_P, K_D = 0.3, 0
    SPEED = 8.1  # Maybe there is a better way to determine speed?
               # Or maybe this is fine since we need to limit speed a bit
               # while handling and avoiding obstacles.
    MIN_OBSTACLE_DIST = 14
    WHEELBASE = 3.0
    TRACK = 1.6
    MAX_MARKERS = 10
    prev_error = 0
    marker_array = MarkerArray()

    def __init__(self):
        self.state_lock = Lock()
        self.goal_pos = (-99.5728759766, -33.2555503845) # We need a goal pose to do
                               # obstacle avoidance. Let's assume that this is initialized
                               # in the form (x, y) from some irrelevant origin.
        self.odom_pose = None  # This is to know which gaps/safe paths to favor
                               # in order to reach goal_pos. -TODO
        self.lidar_sub = rospy.Subscriber("/tesse/hood_lidar/scan", LaserScan, self.lidar_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber("/pf/pose/odom", Odometry, self.odom_callback)
        self.drive_pub = rospy.Publisher("/tesse/drive", AckermannDriveStamped, queue_size=1)
        self.marker_publisher = rospy.Publisher('markers', MarkerArray, queue_size=1)

    # some callbacks in here..

    def odom_callback(self, data):
        self.state_lock.acquire()
        self.odom_pose = data.pose
        self.state_lock.release()

    def lidar_callback(self, data):
        if self.goal_pos is None or self.odom_pose is None:
            return  # We can't move without a goal or car position info

        # get points in the range between -90 and 90 degrees in front of the car
        rt_range = np.array([np.array([data.ranges[i], data.angle_min + i * data.angle_increment])
                            for i in range(len(data.ranges))
                            if (-np.pi/2) <= (data.angle_min + i * data.angle_increment) <= (np.pi/2)])

        # Obstacle dilation to eliminate narrow gaps
        bit_map = [int(p[0] < self.MIN_OBSTACLE_DIST) for p in rt_range]
        prev_obst = -1
        for i in range(len(bit_map)):
            if bit_map[i] == 1:
                prev_obst = i
                continue
            if prev_obst == -1:
                continue
            d_prev_obst = 2*rt_range[prev_obst][0]*np.sin((i-prev_obst)*data.angle_increment/2)
            if d_prev_obst < self.TRACK:  # probably needs tweaking. We only need half of
                                          # the car track but we do extra dilation for safety.
                bit_map[i] = 1  # dilate

        prev_obst = -1
        for i in range(len(bit_map)-1, -1, -1):
            if bit_map[i] == 1:
                prev_obst = i
                continue
            if prev_obst == -1:
                continue
            d_prev_obst = 2*rt_range[prev_obst][0]*np.sin((prev_obst-i)*data.angle_increment/2)
            if d_prev_obst < self.TRACK:
                bit_map[i] = 1

        safe = [rt_range[i] for i in range(len(rt_range)) if bit_map[i] == 1]

        # for group in obstacles:
        #     if len(group) > 4:
        #         xy_range = np.array([np.array([p[0] * np.cos(p[1]), p[0] * np.sin(p[1])]) for p in group])
        #         self.make_marker(xy_range)

        car_pos = self.odom_pose.pose.position
        car_orientation = tf.transformations.euler_from_quaternion((self.odom_pose.pose.orientation.x,
                                                                   self.odom_pose.pose.orientation.y,
                                                                   self.odom_pose.pose.orientation.z,
                                                                   self.odom_pose.pose.orientation.w))
        orientation = car_orientation[2]
        if orientation > 0:
            orientation -= np.pi
        else:
            orientation += np.pi
        ideal_theta = (orientation - np.arctan2(car_pos.y - self.goal_pos[1],
                                                      car_pos.x - self.goal_pos[0]))

        best_gap, heuristic = [], np.inf
        for gap in safe:
            g_theta = ideal_theta - gap[1]
            if abs(g_theta) < heuristic:
                best_gap = gap
                heuristic = abs(g_theta)
        if best_gap == []:
            steering_angle = ideal_theta
        else:
            # print("ideal " + str(ideal_theta))
            # print("best " + str( best_gap[1]))
            target_angle = best_gap[1]
            derivative = target_angle - self.prev_error
            self.prev_error = target_angle
            steering_angle = (self.K_P * target_angle + self.K_D * derivative)
            # steering_angle = ideal_theta
            # steering_angle = self.wall_checker(data, steering_angle)

        print(steering_angle)
        # Publish drive command
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()
        ack_msg.drive.steering_angle = steering_angle
        ack_msg.drive.speed = self.SPEED

        self.drive_pub.publish(ack_msg)

    def wall_checker(self, data, str_angle):
        # Check 3 points directly in front of the robot (theta=0)
        mid_index = int((len(data.ranges) - 1) / 2.0)
        dist_sum = 0
        for i in range(-3, 4):
            dist_sum += data.ranges[mid_index + i]
        avg_dist = dist_sum/3
        # avg_dist_to_front_wall = (data.ranges[mid_index - 1] + data.ranges[mid_index] + data.ranges[mid_index + 1]) / 3.0
        new_str_angle = str_angle
        # The car needs ~0.62m to turn. Adding 0.7 for safety.
        if (avg_dist < self.MIN_OBSTACLE_DIST):
            print('obstacle detected')
            new_str_angle = -np.pi / 2
        return new_str_angle

    def make_marker(self, points):
        marker = Marker()
        marker.header.frame_id = "hood_lidar"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.lifetime = rospy.Duration(3)

        # marker scale
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # marker color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 0

        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        # marker line points
        marker_points = []
        for point in points:
            marker_point = Point()
            marker_point.x = point[0]
            marker_point.y = point[1]
            marker_points.append(marker_point)
        marker.points = marker_points
        # Publish the Marker
        if (len(self.marker_array.markers) > self.MAX_MARKERS):
            self.marker_array.markers.pop(0)
        self.marker_array.markers.append(marker)

        id = 0
        for m in self.marker_array.markers:
            m.id = id
            id += 1

        self.marker_publisher.publish(self.marker_array)
    def avoid_obstacle(self):
        pass

if __name__ == "__main__":
    # initialize node or something and run some function for
    # obstacle avoidance
    rospy.init_node("fast_obstacle")  # maybe change node name?
    fo = FastObstacle()
    rospy.spin()
    