#!/usr/bin/python

import rospy
rospy.init_node('first_node')
rate=rospy.Rate(2)
#defined variable rate which imports Rate func from rospy dependency and set it 2 secs

#infinite loop until programm is killed
#for rate variable function the loop is put to sleep. what does this mean??
while not rospy.is_shutdown(): 
    print "This is my first node"
    rate.sleep()