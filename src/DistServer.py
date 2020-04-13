#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from RBE595_Final_Project.srv import GetDist

class DistServer():
    def __init__(self):
        rospy.init_node('distance_server')
        print("Distance server node has been initialized")

        self.server = rospy.Service('image_dist_service', GetDist, self.calcDist)
        print("Distance server is now running")
        rospy.spin()

    def calcDist(self, data):
        # Get the images out of the data message
        left_img = data.leftImage
        right_img = data.rightImage
        print("Received two images")
        # Do the calculation
        # Return test integer for distance for now
        dist = 2
        print("Distance:", dist)
        return dist

if __name__ == "__main__":
    dist_server = DistServer()
