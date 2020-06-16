#!/usr/bin/env python2
import rospy
from lab4.msg import cone_location, parking_error
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        rospy.Subscriber("/relative_cone", cone_location, 
            self.relative_cone_callback)    
        self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", 
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
            parking_error, queue_size=10)

        self.relative_x = 0
        self.relative_y = 0
    	self.steer_error_n1 = 0 #set initial previous error to zero
    	self.distance_error_n1 = 0 #set initial previous distance error to zero
    	#gains for steering_angle
    	self.kp = 1 
    	self.kd = 0
    	#gains for velocity
    	self.kp_1 = 1
    	self.kd_1 = 0
        self.cone_pos_x = []
        self.cone_pos_y = []
        self.mv_avg_length = 10

    def relative_cone_callback(self, msg):
        self.cone_pos_x.insert(0, msg.x_pos)
        self.cone_pos_y.insert(0, msg.y_pos)
        if len(self.cone_pos_x) > self.mv_avg_length:
            self.cone_pos_x.pop()
            self.cone_pos_y.pop()

        num_points = len(self.cone_pos_x)
        self.relative_x = sum(self.cone_pos_x) / num_points
        self.relative_y = sum(self.cone_pos_y) / num_points
        # self.cone_pos_x.insert(0, msg.x_pos)
        # self.cone_pos_y.insert(0, msg.y_pos)
        # if len(self.cone_pos_x) > 100:
        #     self.cone_pos_x = self.cone_pos_x[:-1]
        #     self.cone_pos_y = self.cone_pos_y[:-1]
        # num_points = 10 #number of points to use in the moving average
        # if len(self.cone_pos_x) < num_points:
        #     self.relative_x = msg.x_pos
        #     self.relative_y = msg.y_pos 
        # else:
        #     self.relative_x = 0
        #     self.relative_y = 0
        #     for i in range(num_points):
        #         self.relative_x += self.cone_pos_x[num_points]/(num_points + 0.0)
        #         self.relative_y += self.cone_pos_y[num_points]/(num_points + 0.0)
        drive_cmd = AckermannDriveStamped()
        
        #################################
        # Play with this number too
        parking_distance = 1.2 #meters
        
        # Your Code Here.
        # Use relative position and your control law to populate
        # drive_cmd.
	""" 
        if self.relative_y > 0:
	    theta = np.arctan(self.relative_x/self.relative_y)
	elif self.relative y < 0:
	    theta = np.arctan(self.relative_x/self.relative_y) + np.pi
	else:
	    if self.relative_x > 0:
	        theta = np.pi/2
	    elif self.relative x < 0:
		theta = - np.pi/2
	    else:
		theta = 0
	eta = 
	if np.sqrt(self.relative_x**2 + self.relative_y**2) > parking_distance + eta:
	    drive_cmd.drive.steering_angle = theta
	    drive_cmd.drive.speed = 1 
	elif np.sqrt(self.relative_x**2 + self.relative_y**2) < parking_distance:
	    drive_cmd.steering_angle = -theta
	    drive_cmd.drive.speed = 1 
	else:
	    pass
	"""
    	drive_cmd.header.frame_id = "base_link"
    	drive_cmd.header.stamp = rospy.get_rostime()
    	drive_cmd.drive.speed = 0.5
    	#work on improving error term
	
        if self.relative_x < 0:
            steering_angle = 3.14
            velocity = 0.5
            distance_error = 0
            steer_error = 0
        else:
            distance_error = np.sqrt(self.relative_y**2 + self.relative_x**2) - parking_distance
    	    steer_error = np.arctan2(self.relative_y, self.relative_x)
    	    steering_angle = self.kp*steer_error + self.kd*(steer_error - self.steer_error_n1)
            velocity = 0.5
            # velocity = 0.5 * np.sign(distance_error)
    	    # velocity = self.kp_1 * distance_error if abs(distance_error < 0.5) else self.kp_1 * 0.5 * np.sign(distance_error)

	   #check for direction of velocity to adjust steering_angle
    	if velocity < 0:
    	    drive_cmd.drive.steering_angle = -steering_angle
    	else:
    	    drive_cmd.drive.steering_angle = steering_angle		
            
	#set maximum velocity so car doesn't buck
        if velocity > 1:
            velocity = 1
        elif velocity < -1:
            velocity = -1

        #limit maximum acceleration
#        drive_cmd.drive.acceleration = 0.001
#        drive_cmd.drive.jerk = 0.01 #m/s^2
        drive_cmd.drive.speed = velocity
    	self.distance_error_n1 = distance_error
    	self.steer_error_n1 = steer_error
	#################################
        self.drive_pub.publish(drive_cmd)
        self.error_publisher()
        
    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = parking_error()
        
        #################################
        
        # Your Code Here
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)
    	error_msg.x_error = self.relative_x
    	error_msg.y_error = self.relative_y
    	#cartesian distance b/n car and cone
    	cart_distance = np.sqrt(self.relative_x**2 + self.relative_y**2)
        error_msg.distance_error = cart_distance
	#################################
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
