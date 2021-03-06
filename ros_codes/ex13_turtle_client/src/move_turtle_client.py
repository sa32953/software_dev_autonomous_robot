#! /usr/bin/env python

import rospy
import time
import actionlib
from ex12_turtle_as.msg import MoveAction, MoveGoal, MoveFeedback, MoveResult
#from actionlib_tutorials.msg import FibonacciAction, FibonacciGoal, FibonacciFeedback, FibonacciResult

def callback(feedback):
    print('Feedback received: ')
    print(feedback)

rospy.init_node('turtle_client')

client = actionlib.SimpleActionClient('move_server', MoveAction)
client.wait_for_server()

goal = MoveGoal()
goal.direction = 'FORWARD'
goal.duration=10

client.send_goal(goal, feedback_cb = callback)

#time.sleep(5)
#client.cancel_goal()

client.wait_for_result()
state_result = client.get_state()

#while state_result < 2:
#    rospy.loginfo("Doing other stuff while the action server does its thing...")
#    state_result = client.get_state()
#    time.sleep(1)


print('[Result] State: %d' %state_result)
