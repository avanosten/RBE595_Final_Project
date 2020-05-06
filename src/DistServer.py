#!/usr/bin/env python
import sys
# I had to remove ros from my path because it was grabbing the wrong cv2 package and then readd it
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy
import rospy
import numpy as np
import math
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from RBE595_Final_Project.srv import GetDist

class DistServer():
    def __init__(self):
        # bridge to convert ros image messages to opencv images
        self.bridge = CvBridge()
        self.cam_dist = 0.7 # distance between the cameras in meters
        self.cam_angle = 60*math.pi/180 # camera perspective angle
        rospy.init_node('distance_server')
        print("Distance server node has been initialized")

        self.server = rospy.Service('image_dist_service', GetDist, self.calcDist)
        print("Distance server is now running")
        rospy.spin()

    def calcDist(self, data):
        # Get the images out of the data message
        left_img = data.leftImage
        right_img = data.rightImage
        
        # Try to convert the images to cv2 images
        try :
            cv_left_img= self.bridge.imgmsg_to_cv2(left_img, desired_encoding='passthrough')
            cv_right_img = self.bridge.imgmsg_to_cv2(right_img, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)

        print("Received two images")
        # Do the calculation
        # Return test integer for distance for now
        img_left = cv2.flip(cv_left_img,0)
        img_right = cv2.flip(cv_right_img,0)
        
        res_w = img_left.shape[1]

        gray_left = cv2.cvtColor(img_left,cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(img_right,cv2.COLOR_BGR2GRAY)

        # get 4 strongest corners for the left and right images
        corners_left = cv2.goodFeaturesToTrack(gray_left,4,0.01,20)
        corners_right = cv2.goodFeaturesToTrack(gray_right,4,0.01,20)

        # convert to numpy array
        corners_left = np.int0(corners_left)
        corners_right = np.int0(corners_right)

        # create arrays for storing the indeces of the corners
        leftCornerIdxs = np.zeros((len(corners_left),2))
        rightCornerIdxs = np.zeros((len(corners_right),2))

        # storing distances of each corner pair array
        dists = np.zeros((len(corners_left)))

        # for every corner match, calculate the distance using distance formula from source cited in our paper
        # I tried getting matching to work properly but for now it doesn't.
        for i in range(0,len(corners_left)):
            xl,yl = corners_left[i].ravel()
            xr,yr = corners_right[i].ravel()
            leftCornerIdxs[i][0] = xl
            leftCornerIdxs[i][1] = yl
            rightCornerIdxs[i][0] = xr
            rightCornerIdxs[i][1] = yr
            D = (cam_dist*res_w)/(2*math.tan((cam_angle/2))*abs(xl-xr))
            dists[i] = D
            cv2.circle(img_left,(xl,yl),5,255,-1)
            cv2.circle(img_right,(xr,yr),5,255,-1)
        
        # the array to return should also take the intensity into account for EKF SLAM recognition
        # needs to be added if there's time
        print("Distances:", dists)
        return dists

if __name__ == "__main__":
    dist_server = DistServer()
