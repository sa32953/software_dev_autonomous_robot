#! /usr/bin/env python

import rospy
import actionlib
from actionlib_tutorials.msg import FibonacciAction, FibonacciGoal, FibonacciFeedback, FibonacciResult
import time

def callback(goal):
    r = rospy.Rate(2)
    success = True

    feedback.sequence = []
    feedback.sequence.append(0)
    feedback.sequence.append(1)

    rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (action_server_name, goal.order, feedback.sequence[0], feedback.sequence[1]))

    for i in range(1, goal.order):
        if action_server.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % action_server_name)
            action_server.set_preempted()
            success = False
            break
        feedback.sequence.append(feedback.sequence[i] + feedback.sequence[i - 1])
        action_server.publish_feedback(feedback)
        time.sleep(0.5)

    if success:
        result.sequence = feedback.sequence
        rospy.loginfo('%s: Succeeded' % action_server_name)
        action_server.set_succeeded(result)

rospy.init_node('fibonacci')

while not rospy.is_shutdown():

    action_server = actionlib.SimpleActionServer("fibonacci_server", FibonacciAction, callback, auto_start = False)
    action_server_name = "Fibonacci Action Server"
    action_server.start()
    print('running')
    feedback = FibonacciFeedback()
    result = FibonacciResult()
    rospy.sleep(2)

