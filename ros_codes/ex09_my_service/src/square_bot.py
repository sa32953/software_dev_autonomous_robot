#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('square_move') # Initiate a Node named 'vel_publisher'
rate=rospy.Rate(0.1) # Set a publish rate of 2 Hz

pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1) # Create a Publisher object, that will publish
t=Twist() #Create the t variable

while not rospy.is_shutdown():
    #move straight and stop
    t.linear.x=0.0157
    pub.publish(t)
    rate.sleep()
    t.linear.x=0
    pub.publish(t)
    rate.sleep()

    #turn 90 degree
    t.angular.z=0.157
    pub.publish(t)
    rate.sleep()
    t.angular.z=0
    pub.publish(t)
    rate.sleep()

