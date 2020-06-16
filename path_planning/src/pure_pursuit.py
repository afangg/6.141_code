#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils
import tf

from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry 
from std_msgs.msg import Float32
from threading import Lock 

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.odom_topic       = rospy.get_param("~odom_topic", "/odom")
        self.odom_pose        = None #store car pose
        self.lookahead_target = 0.7 #target lookahead, tuned for speed of 1.25
        self.lookahead        = self.lookahead_target #current lookahead value. Increases if no path found.
        self.speed            = 1.25 #set arbitrairly
        self.wrap             = 1 #not used
        self.wheelbase_length = 1 #not used
        self.state_lock = Lock()
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.error_pub = rospy.Publisher("/error", Float32, queue_size=1)
        self.initial_pose_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.initial_pose_callback, queue_size = 1)

    def odom_callback(self, msg):
        self.state_lock.acquire()
        self.odom_pose = msg.pose
        self.follow_trajectory()
        self.state_lock.release()
    
    def initial_pose_callback(self, msg):
        self.state_lock.acquire()
        self.odom_pose = msg.pose
        self.follow_trajectory()
        self.state_lock.release()

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        #print "Receiving new trajectory:", len(msg.poses), "points"
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)
        
    def follow_trajectory(self):
        
        pts = self.trajectory.points #points along trajectory
        if (len(pts) == 0 or self.odom_pose == None): #empty trajectory or car location not known
            self.pub_drive_command(0, 0)
            return

        car_position = self.odom_pose.pose.position #assuming that self.odom_topic contains the car's position? but not entirely sure
        x_car = car_position.x
        y_car = car_position.y
        roll, pitch, theta_car  = tf.transformations.euler_from_quaternion((self.odom_pose.pose.orientation.x, self.odom_pose.pose.orientation.y, self.odom_pose.pose.orientation.z, self.odom_pose.pose.orientation.w))
        distance_to_segments = [0] * (len(pts)-1) #just initializing array
        cur_point = np.array([x_car, y_car])
        for i in range(len(distance_to_segments)):
            v = np.array(pts[i]) #v is the first point of ith trajectory segment
            w = np.array(pts[i+1]) #w is second/last point of ith  trajectory segment
            
            l2 = np.linalg.norm(w-v)**2
            t = np.dot(cur_point-v, w-v) / l2
            if t<0:
                t=0
            if t>1:
                t=1
            projection = v + t*(w-v)
            distance_to_segments[i] = np.linalg.norm(cur_point-projection)

        min_index = distance_to_segments.index(min(distance_to_segments)) #idx of first point of segment that is closest to car
        if min_index < 0:
            min_index = 0

        self.publish_error(min(distance_to_segments))

        #now find where trajectory intersects with circle around car (lookahead pt); start searching along the traj. segment that is closest to car
        lookahead_pt = np.array([0, 0])
        q = np.array([car_position.x, car_position.y])
        r = self.lookahead #radius of circle around car
        look_fwd = False
        prev_point = None
        prev_point_theta = 0
        while min_index < len(pts)-1:
            p1 = np.array(pts[min_index]) #first pt of current segment
            p2 = np.array(pts[min_index+1])#2nd/last pt of current segment
            v = p2-p1
            a = np.dot(v, v)
            b = 2.0*(np.dot(v, p1-q))
            c = np.dot(p1, p1)+np.dot(q, q)-2.0*(np.dot(p1, q))-r**2
            disc = b**2-4.0*a*c #discriminant
            if disc >=  0: #if disc is < 0, segment does not intersect circle
                sqrt_disc = np.sqrt(disc)
                t1 = (-b+sqrt_disc)/(2.0*a)
                t2 = (-b-sqrt_disc)/(2.0*a)
                lookahead_pt_poss1 = p1+t1*v #1st intersection pt
                lookahead_pt_poss2 = p1+t2*v #2nd intersection pt
                if 0 <= t1 <= 1 and 0 <= t2 <= 1:
                    #in case both intersection pts lie on the current segment, have to choose between them. right now choosing the one that is closer to p2, so the car moves towards the end goal.
                    if np.linalg.norm(lookahead_pt_poss1 - p2) <= np.linalg.norm(lookahead_pt_poss2 - p2):
                        lookahead_pt=lookahead_pt_poss1
                    else:
                        lookahead_pt=lookahead_pt_poss2
                    break
                elif 0 <= t1 <= 1:
                    #only one of the intersection pts lies on current seg
                    #we want the other intersection point (on the next line seg)
                    dx = lookahead_pt_poss1[0] - x_car
                    dy = lookahead_pt_poss1[1] - y_car
                    theta = np.arctan2(dy,dx) - theta_car
                    if theta > np.pi: #keep theta within range (-pi, pi)
                        theta = -2.0*np.pi + theta
                    elif theta < -np.pi:
                        theta = 2.0*np.pi + theta

                    if (theta < -np.pi/2.0 or theta > np.pi/2) and look_fwd == False: #closest point is behind the car; see if there is a further point that is ahead of the car.
                        look_fwd = True
                        prev_point = lookahead_pt_poss1
                        prev_point_theta = theta
                    elif look_fwd == True: #already looked forward
                        #find which point yields theta closer to zero
                        if (np.abs(prev_point_theta) - np.abs(theta)) > 0: #prev point farther from zero, use new point
                            lookahead_pt=lookahead_pt_poss1
                        else:
                            lookahead_pt=prev_point
                        break
                    else: #found a single point in front ot he car; what we want!
                        lookahead_pt = lookahead_pt_poss1
                        break
                elif 0 <= t2 <= 1:
                    #only one of the intersection pts lies on current seg
                    dx = lookahead_pt_poss2[0] - x_car
                    dy = lookahead_pt_poss2[1] - y_car
                    theta = np.arctan2(dy,dx) - theta_car
                    if theta > np.pi: #keep theta within range (-pi, pi)
                        theta = -2.0*np.pi + theta
                    elif theta < -np.pi:
                        theta = 2.0*np.pi + theta

                    if (theta < -np.pi/2.0 or theta > np.pi/2) and look_fwd == False: #closest point is behind the car; see if there is a further point that is ahead of the car.
                        look_fwd = True
                        prev_point = lookahead_pt_poss2
                        prev_point_theta = theta
                    elif look_fwd == True: #already looked forward
                        #find which point yields theta closer to zero
                        if (np.abs(prev_point_theta) - np.abs(theta)) > 0: #prev point farther from zero, use new point
                            lookahead_pt=lookahead_pt_poss2
                        else:
                            lookahead_pt=prev_point
                        break
                        #t2 is the point then exit loop
                    else: #found a single point in front ot he car; what we want!
                        #this was originally a bug (should look for p1, not p2), but turns out it helps the robot not go backwards on a path. I'm leaving it in.
                        lookahead_pt = lookahead_pt_poss1
                        break

                #else: neither t1 nor t2 are between 0 and 1, so keep looking
                    #keep going through loop 
            min_index = min_index+1 #no lookahead pt found yet, so continue the loop/search for intersection pt with next segment on the trajectory

        if (lookahead_pt[0] == 0 and lookahead_pt[1] == 0): #no lookahead pt found
            #is car very near the end point? Stop the car!
            if np.linalg.norm(np.array([x_car, y_car]) - np.array(pts[len(pts)-1])) < 0.25:
                #print("Less than 0.25m from end point, stopping.")
                self.pub_drive_command(0, 0)
            #car is close by, but not at end point.
            elif (np.linalg.norm(np.array([x_car, y_car]) - np.array(pts[len(pts)-1])) < self.lookahead) and (np.linalg.norm(np.array([x_car, y_car]) - np.array(pts[len(pts)-1])) < np.linalg.norm(np.array([x_car, y_car]) - np.array(pts[0]))):
                dx = pts[len(pts)-1][0] - x_car
                dy = pts[len(pts)-1][1] - y_car
                theta = np.arctan2(dy,dx) - theta_car
                if theta > np.pi: #keep theta within range (-pi, pi)
                    theta = -2.0*np.pi + theta
                elif theta < -np.pi:
                    theta = 2.0*np.pi + theta
                self.pub_drive_command(self.speed, theta)

            #the car may also be near the starting point. The line segment then begins in the circle with radius self.lookahead that surrounds the car. This is a special case
            elif np.linalg.norm(np.array([x_car, y_car]) - np.array(pts[0])) < self.lookahead:
                #First, assume line segment is infinite
                m = (pts[1][1] - pts[0][1])/(pts[1][0] - pts[0][0])
                b = pts[0][1] - m*pts[0][0]
                r = self.lookahead
                target = [0,0]
                
                #Find two points that intersect with circle around car
                x1 = 0
                x2 = 0
                if m*m*b*b - (1 + m*m)*(b*b - r*r) >= 0:
                    x1 = (-m*b + np.sqrt(m*m*b*b - (1 + m*m)*(b*b - r*r)))/(1 + m*m)
                    x2 = (-m*b - np.sqrt(m*m*b*b - (1 + m*m)*(b*b - r*r)))/(1 + m*m)
                y1 = m*x1 + b
                y2 = m*x2 + b

                #Find the one that is closer to p2
                if np.linalg.norm(np.array([x1, y1]) - np.array(pts[1])) <  np.linalg.norm(np.array([x2, y2]) - np.array(pts[1])): #x1, y1 is closer
                    target[0] = x1
                    target[1] = y1
                else:
                    target[0] = x2
                    target[1] = y2

                dx = target[0] - x_car
                dy = target[1] - y_car
                theta = np.arctan2(dy,dx) - theta_car - np.pi
                if theta > np.pi: #keep theta within range (-pi, pi)
                    theta = -2.0*np.pi + theta
                elif theta < -np.pi:
                    theta = 2.0*np.pi + theta
                #print('Starting point:', theta)
                self.pub_drive_command(self.speed, theta)
           
            else:
                self.lookahead += 0.1
                #print("No lookahead point found. Increasing circle radius to ", self.lookahead)
                self.pub_drive_command(0, 0)

        else:
            #curvature = (2*(lookahead_pt[0]-x_car))/(self.lookahead)**2 #curvature of arc between car and lookahead pt
            dx = lookahead_pt[0] - x_car
            dy = lookahead_pt[1] - y_car
            theta = np.arctan2(dy,dx) - theta_car
            if theta > np.pi: #keep theta within range (-pi, pi)
                theta = -2.0*np.pi + theta
            elif theta < -np.pi:
                theta = 2.0*np.pi + theta
            #print("Steering angle (deg): ", theta*180.0/np.pi)
            self.pub_drive_command(self.speed, theta)
            if (self.lookahead > self.lookahead_target):
                self.lookahead -= 0.1

    def pub_drive_command(self, vel, steer_ang):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.frame_id = "base_link"
        drive_msg.header.stamp = rospy.get_rostime()
        drive_msg.drive.speed = vel
        drive_msg.drive.steering_angle = steer_ang
        self.drive_pub.publish(drive_msg)

    def publish_error(self, min_distance):
        error_msg = Float32()
        error_msg.data = min_distance
        self.error_pub.publish(error_msg)

if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
