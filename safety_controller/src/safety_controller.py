#!/usr/bin/env python2

import numpy as np

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class SafetyController:
    SCAN_TOPIC = rospy.get_param("safety_controller/scan_topic")
    # Topic published to by wall follower code
    DRIVE_INTERCEPT_TOPIC = rospy.get_param("safety_controller/drive_intercept_topic")
    # Topic for safety measures to protect the car
    SAFETY_TOPIC = rospy.get_param("safety_controller/safety_topic")

    scan_subscriber = None
    drive_intercept_subscriber = None
    safety_publisher = None

    # Intercepted data
    scan_data = None
    drive_data = None

    # Time stamps of when the data where received. Assigned separately to
    # avoid user mistakes.
    scan_stamp = None
    drive_stamp = None

    # Minimum distance to be maintained from front and side obstacles. If speed
    # is greater than a certain threshold, these bounds grow to account for extra
    # time/distance required for braking.
    MIN_SIDE = 0.25
    MIN_FRONT = 0.4

    # To avoid false positives, we require a minimum of 5 points within the
    # danger rectangles.
    MIN_DANGER_POINTS = 3

    def __init__(self):
        # Initialize publishers and interception subscribers.
        self.safety_publisher = rospy.Publisher(self.SAFETY_TOPIC, AckermannDriveStamped, queue_size=10)
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.save_scan_callback)
        rospy.Subscriber(self.DRIVE_INTERCEPT_TOPIC, AckermannDriveStamped, self.safety_callback)

    def save_scan_callback(self, data):
        self.scan_stamp = rospy.Time.now()
        self.scan_data = data

    def safety_callback(self, data):
        self.drive_stamp = rospy.Time.now()
        self.drive_data = data
        scan = self.scan_data

        if scan == None:
            return

        vel = data.drive.speed
        min_front_angle = - np.pi / 2
        max_front_angle = np.pi / 2
        dist_range = scan.ranges

        min_side = max(self.MIN_SIDE, vel / 10.0)
        min_front = max(self.MIN_FRONT, vel / 3.0)

        rt_range = [np.array([dist_range[i], scan.angle_min + i*scan.angle_increment])
                                for i in range(len(dist_range))]
        front_rt_points = list(filter(lambda x : min_front_angle <= x[1] <= max_front_angle, rt_range))

        danger_points = list(filter(lambda x : self.is_danger_point(x, min_side, min_front), front_rt_points))

        if (len(danger_points)) > self.MIN_DANGER_POINTS:
            rospy.loginfo("%d danger points spotted!", len(danger_points))
            self.send_stop_command()

    def is_danger_point(self, rt_point, min_side, min_front):
        # Check whether rt_point is within the danger rectangular bound.
        dist, theta = rt_point[0], abs(rt_point[1])
        return dist < (min_side / np.sin(theta)) and dist < (min_front / np.cos(theta))

    def send_stop_command(self):
        # Send stopping command.
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()
        ack_msg.drive.steering_angle = 0.0
        ack_msg.drive.speed = 0.0

        rospy.loginfo("Stop command issued!")
        self.safety_publisher.publish(ack_msg)


if __name__ == "__main__":
    rospy.init_node('safety_controller')
    sc = SafetyController()
    sleep_rate = rospy.Rate(15) # sleep for some time
    safety_dt = rospy.Duration(secs=0.4)

    while not rospy.is_shutdown():
        # Handle interruptions to LIDAR and wall following nodes.
        now = rospy.Time.now()
        drive_stamp = sc.drive_stamp if sc.drive_stamp != None else None
        scan_stamp = sc.scan_stamp if sc.scan_stamp != None else None

        if (drive_stamp != None and (now - drive_stamp) > safety_dt) or \
            (scan_stamp != None and (now - scan_stamp) > safety_dt):
            sc.send_stop_command()
            
        sleep_rate.sleep()
