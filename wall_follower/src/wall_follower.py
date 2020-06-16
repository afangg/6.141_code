#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    # DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")

    # Subscriber to LIDAR scan data and publisher for drive
    # commands.
    scan_subscriber = None
    drive_publisher = None

    DEFAULT_STEER_ANGLE = 0.0

    integral = 0.0
    previous_error = 0.0

    k_p = 5
    k_d = 3
    k_i = .1

    def __init__(self):
        # TODO:
        # Initialize your publishers and
        # subscribers here

        self.drive_publisher = rospy.Publisher('drive', AckermannDriveStamped, queue_size=10)
        self.marker_publisher = rospy.Publisher('marker', Marker, queue_size=10)
        self.marker = Marker()
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.steer_callback)

    # TODO:
    # Write your callback functions here.

    """
    This callback function represents the tip of a series
    of functions to process LIDAR scan data, produce control
    commands, and publish steer commands to DRIVE_TOPIC to
    move the car.


    data: LIDAR scan data of type LaserScan.
    """
    def steer_callback(self, data):
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()
        ack_msg.drive.steering_angle = 0.0
        ack_msg.drive.speed = self.VELOCITY

        # rospy.loginfo(rospy.get_caller_id() + "steering with %f m/s speed", self.VELOCITY)

        # np array of length 3 containing (m, c) values for each
        # of the right, forward, and left wall lines.
        closest_index = 0 if (self.SIDE == -1.0) else 1
        # lines = self.find_two_wall_lines(data)
        sliced_data = self.slice_laser_points(data)
        if (len(sliced_data[closest_index]) > 1):

            # Find distance to each wall
            # dists = np.array([(np.abs(lines[i][1]) / np.sqrt((lines[i][0]**2) + 1.0)) for i in range(len(lines))])
            # walls = ["right", "front", "left"]
            # right_dist = np.abs(lines[0][1]) / np.sqrt((lines[0][0]**2) + 1.0)
            # front_dist = np.abs(lines[1][1]) / np.sqrt((lines[1][0]**2) + 1.0)
            # left_dist = np.abs(lines[2][1]) / np.sqrt((lines[2][0]**2) + 1.0)
            # if dists[closest_index] > dists[1]:
            #     closest_index = 1


            m, c = self.find_lq_line(sliced_data[closest_index])
            closest_dist = np.abs(c) / np.sqrt((m**2) + 1.0)
            steering_angle = self.find_steering_angle(m, c, closest_dist)
            self.make_marker(m, closest_dist)
	        #[new] check if there is a wall directly ahead
            steering_angle = self.wall_checker(data, steering_angle)

            ack_msg.drive.steering_angle = steering_angle

        # rospy.loginfo("distances: %f,  %f,  %f", dists[0], dists[1], dists[2])

        # rospy.loginfo("wall followed: " + walls[closest_index])
        self.drive_publisher.publish(ack_msg)

    """
    Slices LaserScan data into 3 ranges in a 3D np array, one
    range for each third of the total angle LIDAR covers. Each
    2D inner range array contains points in (x, y) coordinates,
    using the LIAD location as the origin.


    data: LIDAR scan data of type LaserScan.
    """
    def slice_laser_points(self, data):
        dist_range = np.array(data.ranges)
        # angle_range = np.arange(data.angle_min,
        #                         data.angle_max,
        #                         data.angle_increment)
        rt_range = np.array([np.array([dist_range[i], data.angle_min + i * data.angle_increment])
                            for i in range(len(dist_range))])
        splits = np.array_split(rt_range, 2)
        splits_filtered = [list(filter(lambda x : x[0] < 3, split)) for split in splits]

        # rt_range_filtered = list(filter(lambda x : x[1] > 3, rt_range))

        xy_range = np.array([np.array([np.array([p[0] * np.cos(p[1]), p[0] * np.sin(p[1])]) for p in split])
                                for split in splits_filtered])
        # xy_range = np.array([[dist_range[i] * np.cos(data.angle_min + i * data.angle_increment), 
        #                       dist_range[i] * np.sin(data.angle_min + i * data.angle_increment)]
        #                       for i in range(len(dist_range))])
        # Split into 3 ranges: [left, front, right]
        # return np.array_split(xy_range, 3)
        return xy_range

    """
    Returns the least squares best-fit line for a set of xy-points.


    xy_points: (n x 2) np array of n 2D points, where each point is
               a row.

    return: the tuple (m, c), where: y = m*x + c.
    """
    def find_lq_line(self, xy_points):
        x = xy_points[:,0]
        y = xy_points[:,1]

        return np.polyfit(x, y, 1)

    """
    Splits the data into 3 near-equal pieces and uses least squares
    regression to find the best-fit line for each chunk of data.


    data: LIDAR scan data of typ LaserScan.

    return: np array of the form [[m1, c1], [m2, c2], [m3, c3]]
    """
    def find_two_wall_lines(self, data):
        sliced_data = self.slice_laser_points(data)
        return np.array([self.find_lq_line(data_chunk) for data_chunk in sliced_data])

    """
    Uses PID control to find the optimal steering angle in order to
    follow the desired wall characterized by: y = m*x + c at the desired
    side.


    m and c: slope and y-intercept, respectively.
    distance: distance from the car to the specified line.

    return: steering angle, in radians.
    """
    def find_steering_angle(self, m, c, distance):
        steer_angle = 0.0
        # if self.SIDE == np.sign(c):
        error = distance - self.DESIRED_DISTANCE
        # self.integral = self.integral + error
        derivative = (error - self.previous_error)
        steer_angle = np.arctan(self.k_p * error +
                                self.k_i * self.integral +
                                self.k_d * derivative)*self.SIDE + np.arctan(m)
        self.previous_error = error

        # rospy.loginfo("error: %f,  steer angle: %f,  m: %f", error, steer_angle, m)
        # else:
        #     self.integral = 0.0
        #     self.previous_error = 0.0
        #     steer_angle = (np.pi / 2.0) * self.SIDE

        return steer_angle

    def wall_checker(self, data, str_angle):
        # Check 3 points directly in front of the robot (theta=0)
        mid_index = int((len(data.ranges)-1)/2.0)
	avg_dist_to_front_wall = (data.ranges[mid_index-1] + data.ranges[mid_index] + data.ranges[mid_index+1])/3.0
	new_str_angle = str_angle
        #The car needs ~0.62m to turn. Adding 0.7 for safety.
	if (avg_dist_to_front_wall < self.DESIRED_DISTANCE + 0.7):
            if (self.SIDE == 1): #following left wall, turn RIGHT
                new_str_angle = -np.pi/2
            else: #following right wall, turn LEFT
                new_str_angle = np.pi/2
        return new_str_angle

    def make_marker(self, m, dist):
        self.marker.header.frame_id = "base_link"
        self.marker.type = self.marker.LINE_STRIP
        self.marker.action = self.marker.ADD
        self.marker.lifetime = rospy.Duration(3)

        # marker scale
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1

        # marker color
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0

        # marker orientaiton
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 0

        # marker position
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = dist * self.SIDE
        self.marker.pose.position.z = 0.0

        # marker line points
        points = []
        increment = 0.2
        for i in range(10):
            point = Point()
            x = increment * i
            y = m * x
            point.x = x
            point.y = y
            points.append(point)

        self.marker.points = points
        # Publish the Marker
        self.marker_publisher.publish(self.marker)



if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
