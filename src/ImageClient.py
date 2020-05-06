#!/usr/bin/env python
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy
import numpy as np
import rospy
from sensor_msgs.msg import Image
from RBE595_Final_Project.srv import GetDist


class ImageClient():
    def __init__(self):
        # Initialize this goal
        rospy.init_node("image_client")

        # Initialize the images of the stereo cameras
        self.img_left = Image()
        self.img_right = Image()

        # Wait for the service to start before you can start sending requests
        print("Waiting for server...")
        rospy.wait_for_service('image_dist_service')
        print("Connected to server")

        self.distFromImagesProxy = rospy.ServiceProxy('image_dist_service', self.getDistFromImages)
        print("DistanceFromImages proxy is live")

        # Create the subscribers to take in the images from Coppelia, topic names are set in Coppelia 
        left_image_sub = rospy.Subscriber("/left_image", Image, self.updateLeftImage)
        left_image_sub = rospy.Subscriber("/right_image", Image, self.updateRightImage)

        # Run at 10Hz
        self.r = rospy.Rate(10)

    # Callback to update the left image, expecting the data from Coppelia to be a sensor_msgs/Image.msg
    def updateLeftImage(self, data):
        self.img_left = data

    # Callback to update the right image, expecting the data from Coppelia to be a sensor_msgs/Image.msg
    def updateRightImage(self, data):
        self.img_right = data
    
    # Ask Server for distance between images.
    def getDistFromImages(self, data):
        try:
            # Asking for distance from two images from the distance server.
            distance = self.distFromImagesProxy(self.img_left, self.img_right)
            print("Measured distance:", distance)
        except rospy.ServiceException as e:
            print ("Service call failed: ")

# Create the client when you run this script.
client = ImageClient()
# Run for 100 iterations (just for testing, make some kind of while loop later)
for i in range(100):
    client.getDistFromImages()


# img_left = cv2.imread('/home/rhosea/catkin_ws/src/RBE595_Final_Project/src/landscape_left.jpg', 0)
# img_right = cv2.imread('/home/rhosea/catkin_ws/src/RBE595_Final_Project/src/landscape_right.jpg', 0)
# cv2.imshow("landscape_image",img_left)
# cv2.waitKey(0) 
# cv2.destroyAllWindows() 