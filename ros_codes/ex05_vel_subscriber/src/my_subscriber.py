#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

#callback function that calls the msg defined in below statements
def callback(msg):
    print msg.linear.x

#print command can be used to print either the whole msg or a part of msg,...
#...details of msg can be checked by eg rosmsg info geometry_msgs or rosmsg list

rospy.init_node('vel_subscriber') #Intitates node
rospy.Subscriber('/cmd_vel', Twist, callback) #function to define that it should... 
#...subscribe from <this> topic & <this> type of msg
rospy.spin()
