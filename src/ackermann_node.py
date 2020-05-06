#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
class AckermannCar():
    def __init__(self):
        rospy.init_node('ackermann_node')
        self.currSteeringAngle = Float32()
        print("ackermann started")
        self.steerPub = rospy.Publisher('/steering', Float32, queue_size=10)
        self.throttlePub = rospy.Publisher('/throttle', Float32, queue_size=10)
        self.steerSub = rospy.Subscriber('/steering_angle', Float32, self.turn)
        self.cmdVelSub = rospy.Subscriber('/cmd_vel', Twist, self.teleopWrapper)
        rospy.spin()

    def turn(self, data):
        self.currSteeringAngle = data.data
        print (self.currSteeringAngle)
        self.steerPub.publish(self.currSteeringAngle)
    
    def teleopWrapper(self, data):
        self.steerPub.publish(data.angular.z)
        self.throttlePub.publish((data.linear.x)*10)

if __name__ == "__main__":
    ackermann = AckermannCar()