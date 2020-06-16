#!/usr/bin/env python2
import rospy
import cv2
from sensor_msgs.msg import Image
from lab4.msg import cone_location
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from color_seg import cd_color_segmentation
from visualization_msgs.msg import Marker

class ConeMarker():
    """
    Rosnode for handling simulated cone. Listens for clicked point
    in rviz and publishes a marker. Publishes position of cone
    relative to robot for Parking Controller to park in front of.
    """

    def __init__(self):
        # Subscribe to clicked point messages from rviz
        rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.marker_pub = rospy.Publisher("/cone_marker", Marker, queue_size = 10)
        self.cone_frame = None
        self.frame_x = None
        self.frame_y = None
        self.bridge = CvBridge()
        self.homography_matrix = self.calibrate()

        self.message_frame = "base_link"

        self.cone_pub = rospy.Publisher("/relative_cone", cone_location, queue_size=1)
        self.rate = rospy.Rate(10)  # 10 hz

        # For line following, change to True
        self.restrict = True

        while not rospy.is_shutdown():
            self.rate.sleep()

    def restrict_line_view(self, img):
        """
        Takes a cv2 image object and blacks out top and bottom portions to
        restrict the view in way that enables line following.

        img: opencv image object.
        returns: img copy with particular portions blacked out.
        """
        h, w, _  = img.shape
        # Tunable
        box_1 = ((0, 0), (w, int((2.5 / 4) * h)))
        box_2 = ((0, int((3.5 / 4) * h)), (w, h))
        first_box = cv2.rectangle(img, box_1[0], box_1[1], (0, 0, 0), -1)
        restricted_img = cv2.rectangle(first_box, box_2[0], box_2[1], (0, 0, 0), -1)

        return restricted_img

    # Apply homography matrix and then publish transformed x,y

    def image_callback(self, msg):
        print('in callback')
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = self.restrict_line_view(image) if self.restrict else image
            temp_bounding_box = cd_color_segmentation(cv_image)
            if temp_bounding_box != ((0, 0), (0, 0)):
                self.cone_frame = cv_image
                midpoint = [(temp_bounding_box[0][0]+temp_bounding_box[1][0])/2, temp_bounding_box[1][1]]
                x, y = self.pixel2dist(midpoint[0], midpoint[1])
                relative_cone = cone_location()
                relative_cone.x_pos = x
                relative_cone.y_pos = y
                if x < 0:
                    print('invalid position of cone; ignoring')
                else:
                    print('orange cone detected')
                    self.cone_pub.publish(relative_cone)
                    self.make_marker(x, y)
            else:
                print('no cone detected')
        except CvBridgeError as e:
            print(e)

    def calibrate(self):
        #Source: image pixels. Top left to bottom right, by column. 1st tuple.
        #Destination: Real-world x, y, and z coordinates relative to the camera. 2nd tuple
        p1 = ( (224,182), (2.875, 1) )
        p2 = ( (166,194), (1.875, 1) )
        p3 = ( (7,222), (0.875+0.10, 1) )
        p4 = ( (342,183), (2.875, 0) )
        p5 = ( (346,197), (1.875, 0) )
        p6 = ( (354,234), (0.875, 0) )
        p7 = ( (462,186), (2.875, -1) )
        p8 = ( (528,196), (1.875, -1) )
        p9 = ( (667,229), (0.875, -1+0.013) )
        pts_src = np.array((p1[0],p2[0],p3[0],p4[0],p5[0],p6[0],p7[0],p8[0],p9[0]))
        pts_dst = np.array((p1[1],p2[1],p3[1],p4[1],p5[1],p6[1],p7[1],p8[1],p9[1]))
        # Calculate Homography
        h, status = cv2.findHomography(pts_src, pts_dst)
        return h

    def pixel2dist(self, x, y):
        px = np.array([[x, y, 1]]).T
        dst = self.homography_matrix.dot(px)
        x_dst = dst[0]/dst[2]
        y_dst = dst[1]/dst[2]
        return [x_dst[0], y_dst[0]]

    def make_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.lifetime = rospy.Duration(3)
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        self.marker_pub.publish(marker)

if __name__ == '__main__':
    try:
        rospy.init_node('ConeMarker', anonymous=True)
        ConeMarker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
