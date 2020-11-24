#! /usr/bin/env python
import rospy
import actionlib
from ex12_turtle_as.msg import MoveAction, MoveGoal, MoveFeedback, MoveResult
from geometry_msgs.msg import Twist
import time


def move_r(sekunden,v):
    #pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10) # Create a Publisher object, that will publish
    #t=Twist() #Create the t variable
    time.sleep(0.5) #Sleep for half second

    t.linear.x=v*0.05 #Set move vel
    pub.publish(t) #Publish to cmd topic
    time.sleep(sekunden) #Wait for goal.duration seconds
    t.linear.x=0 #Set vel to zero
    pub.publish(t) #Publish
   
    

def callback(goal):
    global t
    r = rospy.Rate(2)
    if goal.direction=='FORWARD':
        #MOVE FORWARD FOR goal.duration seconds
        sekunden=goal.duration
        
        #Set the feedback state and publish it
        feedback.current_state='Moving forward!'
        action_server.publish_feedback(feedback)
        
        #Wait for 1 second and move the Robot for defined seconds 'duration'
        t.linear.x=0.05

        #After it has completed moving, set the result state
        result.final_state='Finished moving!'
        action_server.set_succeeded(result)
        
                
    if goal.direction=='BACKWARD':
        #MOVE Backward FOR goal.duration seconds
        sekunden=goal.duration
        
        #Set the feedback state and publish it
        feedback.current_state='Moving backward!'
        action_server.publish_feedback(feedback)
        
        #Wait for 1 second and move the Robot for defined seconds 'duration'
        t.linear.x=-0.05

        #After it has completed moving, set the result state
        result.final_state='Finished moving!'
        action_server.set_succeeded(result)

    if goal.direction!='FORWARD' and goal.direction!='BACKWARD': 
        rospy.loginfo('Incorrect goal: Please specify FORWARD or BACKWARD!')

    r.sleep()
    
    
rospy.init_node('move_robot')
pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10) # Create a Publisher object, that will publish
t=Twist() #Create the t variable
action_server = actionlib.SimpleActionServer("move_server", MoveAction, callback, auto_start = False)
action_server_name = "Move Action Server"

action_server.start()

feedback = MoveFeedback()
result = MoveResult()
print('2')


if t.linear.x != 0:
    pub.publish(t)
    print('1')
    time.sleep(sekunden)

print('3')
rospy.spin()

