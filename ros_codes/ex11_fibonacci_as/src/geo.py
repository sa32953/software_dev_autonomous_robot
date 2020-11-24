#! /usr/bin/env python
import rospy
import actionlib
from actionlib_tutorials.msg import FibonacciAction, FibonacciGoal, FibonacciFeedback, FibonacciResult

def callback(goal):
    r=rospy.Rate(2)
    success=True

    feedback.sequence = []
    feedback.sequence.append(1)

    rospy.loginfo('%s: Executing, creating geometric sequence of order %i with seed %i' % (action_server_name, goal.order, feedback.sequence[0]))
    for i in range(1, goal.order):
        if action_server.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % action_server_name)
            action_server.set_preempted()
            success = False
            break
        
        new_node=feedback.sequence[i-1]+feedback.sequence[i-1]
        feedback.sequence.append(new_node)
        action_server.publish_feedback(feedback)
        r.sleep()
    
    if success:
        result.sequence = feedback.sequence
        rospy.loginfo('%s: Succeeded' % action_server_name)
        action_server.set_succeeded(result)

rospy.init_node('geo')

action_server = actionlib.SimpleActionServer("geo_server", FibonacciAction, callback, auto_start = False)
action_server_name = "Geometric Action Server"

action_server.start()

feedback = FibonacciFeedback()
result = FibonacciResult()

rospy.spin()
