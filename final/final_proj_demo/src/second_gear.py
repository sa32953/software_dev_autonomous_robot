#!/usr/bin/env python

##########################################
##       Created by: SAHIL ARORA        ## 
##  HOCHSCHULE- RAVENSBURG WEINGARTEN   ##
##          Date: 10 Juli 2020          ##
##########################################

import rospy
import actionlib
import math
import numpy
import time

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from goal_publisher.msg import GoalPoint, PointArray

from tf import transformations

#Intializations of variables
goal_subs_count=0

later_goal = []

# This variable is used to detect if the robot is stuck at the same position
# It checks the displacement and orientation of the robot
time_window = 0





############################
##    Get All Targets     ##
############################
#(reused)

def get_target(msg):
    #Takes callback from /goal_publisher package (/goals publisher) and gives target points.
    #Takes 'PointArray' msg which has 'goals' message 
    global all_targets, goal_subs_count

    #Defined to take the msg only once and store it in list all_targets
    if goal_subs_count == 0:
        all_targets=msg.goals
        rospy.loginfo('Targets to Cover: {}'.format(len(all_targets)))
        goal_subs_count = goal_subs_count + 1




############################
##  Get Current Position  ##
############################

def get_current_position(pos):
    #Takes callback from /amcl_pose topic and gives position and orientation.
    global current_x_pos, current_y_pos, current_theta ,err_posn
    
    current_x_pos = pos.pose.pose.position.x
    current_y_pos = pos.pose.pose.position.y
    
    #Define orientation of Robot.
    rot_q = pos.pose.pose.orientation
    quaternions = ( rot_q.x , rot_q.y , rot_q.z , rot_q.w )

    #ROS gives output in quaternion system. need to be converted to Euler angles.
    euler = transformations.euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
    current_theta = euler[2] #Take the z component



############################
##     Main Function      ##
############################

def main():
    global all, goal_subs_count, action_server , err_posn, later_goal, time_window
    global current_x_pos, current_y_pos, current_theta, all_targets, target_now

    rospy.init_node('second_gear')
    time.sleep(0.5)

    #Subscriber that stores all target points
    rospy.Subscriber('/goals', PointArray, get_target)
    rospy.wait_for_message('/goals', PointArray)

    while not rospy.is_shutdown():
        
        # Subscribe to /amcl_pose topic to localize the turtlebot in the given map
        localize = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped , get_current_position)

        # Create an action client called "move_base" with action message "MoveBaseAction"
        move_clnt = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # Waits until the action server has started up and started listening for goals.
        move_clnt.wait_for_server()

        # Creates a new message variable 't' of type MoveBaseGoal
        t = MoveBaseGoal()

        while all_targets != []:
            target_now = all_targets[0]
            #print(target_now)

            #Count Time instances
            if 0 <= time_window <= 0.025:
                look_t0_x= current_x_pos #Register Current posn at t = 0 sec
                look_t0_y= current_y_pos
                yaw_t0 = current_theta
        
            time_window = time_window + 0.02


            # Move to des point in "map" coordinate frame 
            t.target_pose.header.frame_id = "map"
            t.target_pose.header.stamp = rospy.Time.now()
            t.target_pose.pose.position.x = target_now.x
            t.target_pose.pose.position.y = target_now.y
            t.target_pose.pose.orientation.w = 1

            
            # Send goal information
            move_clnt.send_goal(t)
            wait = move_clnt.wait_for_result() #Waiting for result


            
            # Positional Accuracy
            err_posn= math.sqrt(pow((target_now.x-current_x_pos),2) + pow((target_now.y-current_y_pos),2))
            
            if move_clnt.get_state() == 3:
                print('Reached point {}'.format(target_now))
            
                rospy.loginfo('Positional Accuracy = {} mm'.format(err_posn * 1000))
                print('Hey judges ! I am pausing for 1.0 secs. Please note my reward points.')
                time.sleep(1.25)
                
                #rospy.sleep(rate)
                all_targets = all_targets [1:]


            if move_clnt.get_state() == 4:
                print('Goal saved for later. Moving to next target.')
                later_goal.append(target_now)

                all_targets = all_targets [1:]

            
            # Setting Time window of 10 secs
            if 9.9 <= time_window <= 10.1  :
                look_t5_x= current_x_pos
                look_t5_y= current_y_pos
                yaw_t5 = current_theta

                move_in_5_sec = math.sqrt(pow((look_t5_x - look_t0_x),2) + pow((look_t5_y - look_t0_y),2))
                threshold_move= 0.15
                if move_in_5_sec < threshold_move:
                    print('Goal saved for later. Moving to next target.')
                    later_goal.append(target_now)

                    all_targets = all_targets [1:]
                    
                    
                #empty the bucket
                time_window = 0

            


        time.sleep(0.05)

if __name__ == '__main__':
    main()
