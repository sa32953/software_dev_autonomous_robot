#!/usr/bin/env python

import rospy # Import the Python library for ROS
from geometry_msgs.msg import Twist # Import the Twist message from geometry_msgs/msg folder

rospy.init_node('vel_publisher') # Initiate a Node named 'vel_publisher'
rate=rospy.Rate(2) # Set a publish rate of 2 Hz

pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1) # Create a Publisher object, that will publish
t=Twist() #Create the t variable
t.linear.x=0.1 # Initialize 'count' variable

while not rospy.is_shutdown():
    pub.publish(t) # Publish the message within the 'count' variable
    
    rate.sleep() # Make sure the publish rate maintains at 2 Hz
