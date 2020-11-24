#!/usr/bin/env python

import rospy
from goal_publisher.msg import GoalPoint, PointArray

def callback(msg):
    print('Point 0: x: {}, y: {}, z: {}'.format(msg.goals[0].x , msg.goals[0].y , msg.goals[0].z))
    print('Point 1: x: {}, y: {}, z: {}'.format(msg.goals[1].x , msg.goals[1].y , msg.goals[1].z))
    print('Point 2: x: {}, y: {}, z: {}'.format(msg.goals[2].x , msg.goals[2].y , msg.goals[2].z))


rospy.init_node('ex07_node') # Initiate a Node named 'ex07_node'
rospy.Subscriber('/goals', PointArray , callback)
rospy.spin()

