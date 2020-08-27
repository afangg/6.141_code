#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Point, Pose, PoseWithCovariance
import time, os
from utils import LineTrajectory #idk if I imported this correctly
from nav_msgs.msg import Odometry
import rospkg

class PathBuilder(object):

    def __init__(self):

        self.odom_sub = rospy.Subscriber("/pf/pose/odom", Odometry, self.odom_callback)
        self.traj = LineTrajectory()

        rospack = rospkg.RosPack()
        fc_path = rospack.get_path("final_challenge")
        self.save_path = os.path.join(fc_path+"/trajectories/", time.strftime("%Y-%m-%d-%H-%M-%S") + ".traj")

        rospy.on_shutdown(self.traj.save(self.save_path))

    def odom_callback(self, msg):

        point = msg.pose.pose.position #get Point from Odometry msg
        self.traj.addPoint(point)

if __name__=="__main__":
    rospy.init_node("path_builder")
    pf = PathBuilder()
    rospy.spin()



