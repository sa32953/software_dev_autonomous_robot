#!/usr/bin/env python

##########################################
##       Created by: SAHIL ARORA        ## 
##  HOCHSCHULE- RAVENSBURG WEINGARTEN   ##
##          Date: 01 May 2020           ##
##########################################

import rospy
import math
import numpy
from goal_publisher.msg import GoalPoint, PointArray
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from tf import transformations
from sensor_msgs.msg import LaserScan
import time
from ex12_turtle_as.msg import MoveAction, MoveGoal, MoveFeedback, MoveResult
from custom_servicemsg_for_miniproj.srv import StringServiceMsg, StringServiceMsgResponse

#Intializations of variables
curr_posn_x = 0
curr_posn_y = 0
yaw = 0
curr_state = 0
err_yaw = 0
desired_yaw = 0
goal_subs_count=0
switch = 0

#Distance and Orientation precisions
dist_precision = 0.1
yaw_precision= math.pi/360 #around 0.5 degrees



############################
##     Get Laser Scan     ##
############################

def get_laser_scan(msg):
    global regions, safe_dist, state_description
  
    regions = { 'right': min(min(msg.ranges[252:287]), 10), 
    'f_right': min(min(msg.ranges[288:340]), 10), 'front': min(min(msg.ranges[0:17]), 10,min(msg.ranges[341:359])), 
    'f_left': min(min(msg.ranges[18:70]), 10),
    'left': min(min(msg.ranges[71:106]), 10) } 
    #180/5= 36 each

    safe_dist=0.75 #Safe distance to Obstacles in metres

    if regions['front'] > safe_dist and regions['f_left'] > safe_dist and regions['f_right'] > safe_dist :
        #Means nothing is there in Front, FL and FR
        state_description = 1
        change_state(0)
    
    else:
        state_description = 0 
        change_state(3)


############################
##  Get Current Position  ##
############################

def get_current_posn(msg):
    #Takes callback from /gazebo/model_states topic and gives position and orientation.

    global yaw, curr_posn_x, curr_posn_y, quaternions, euler
    
    #Define current posn of Robot in X and Y co-ordinates.
    curr_posn_x=msg.pose[1].position.x
    curr_posn_y=msg.pose[1].position.y

    #Define orientation of Robot.
    quaternions= ( msg.pose[1].orientation.x , msg.pose[1].orientation.y ,
    msg.pose[1].orientation.z ,msg.pose[1].orientation.w )

    #ROS gives output in quaternion system. need to be converted to Euler angles.
    euler=transformations.euler_from_quaternion(quaternions)
    yaw=euler[2] #Take the z component

############################
##    Get All Targets     ##
############################

def get_target(msg):
    #Takes callback from /goal_publisher package (/goals publisher) and gives target points.
    #Takes 'PointArray' msg which has 'goals' message 
    global all_targets,goal_subs_count
    
    #Defined to take the msg only once and store it in list all_targets
    if goal_subs_count == 0:
        all_targets=msg.goals
        rospy.loginfo('Targets to Cover: {}'.format(len(all_targets)))
        goal_subs_count = goal_subs_count + 1
    
############################
##   Set Closest Target   ##
############################

def set_target():

    #Callback function that calculates distance to all pending targets...
    #... and choses with min distance...
    #...and removes the chosen target from the all_targets list

    global dist_to_target, all_targets, target_now
    dist_to_target=[None]*len(all_targets) #Empty List of same length as all_targets list

    for i in range(len(all_targets)):
        dist_to_target[i] = math.sqrt(pow((all_targets[i].x-curr_posn_x),2) + pow((all_targets[i].y-curr_posn_y),2))
        i=i+1
    all_targets = [all_targets for _,all_targets in sorted(zip(dist_to_target,all_targets))]
    # line 114: sorting all targets based on distance

    target_now = all_targets[0] #Target Chosen
    all_targets = all_targets[1:] #Removing chosen target from list
    rospy.loginfo('Target set. Soldier approaching target !!')
    rospy.loginfo('t_x={} and t_y={}'.format(target_now.x,target_now.y))
     
############################
##  Change State of Robot ##
############################

def change_state(state):
    #Callback func to change states of movement

    global curr_state
    curr_state = state

############################
##    Normalize Yaw     ##
############################

def normalize_angle(angle):

    #Callback function to adjust value of err_yaw if it goes beyond 180 degrees
    # if angle is more that 180 degrees then angle = angle -360 (if angle is +ve) otherwise
    # angle = 360 + angle (take care that angle is a -ve value)
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

############################
##        Fix yaw         ##
############################

def fix_yaw(target_now):
    #Function to adjust orientation of Robot
    #Calculated error in desired yaw and current yaw, decides action based on conditions
    global curr_posn_y, curr_posn_x, yaw_precision, err_yaw, desired_yaw, pub, twist_msg
    
    desired_yaw = math.atan2((target_now.y - curr_posn_y), (target_now.x - curr_posn_x))
    err_yaw = normalize_angle(desired_yaw - yaw)

    if abs(err_yaw) > yaw_precision:
        twist_msg.angular.z = 0.3 if err_yaw > 0 else -0.3

    #When precision is reached, then move straight    
    if abs(err_yaw) <= yaw_precision:
        change_state(1)

    # Note: +ve angular vel means anticlockwise movement

############################
##       Move Ahead       ##
############################

def move_ahead(target_now):

    #Function to drive the Robot straight to the set target
    #Check conditions if err_yaw is more than precision

    global err_yaw, yaw, curr_posn_x, curr_posn_y, dist_precision
    global yaw_precision, desired_yaw, pub, twist_msg, state_description, err_posn

    desired_yaw = math.atan2((target_now.y - curr_posn_y), (target_now.x - curr_posn_x))
    err_yaw = desired_yaw - yaw

    err_posn= math.sqrt(pow((target_now.x-curr_posn_x),2) + pow((target_now.y-curr_posn_y),2))
    
    if err_posn > dist_precision:
        twist_msg.linear.x = 0.2
        twist_msg.angular.z = 0     

    # If distance precision is reached it considers as point is reached    
    if err_posn < dist_precision:
        change_state(2) #Reached point

    #Constantly check orientation position and if is decreases then goes adjusting yaw state
    if abs(err_yaw) > yaw_precision:
        change_state(0)

############################
##      Constant Left     ##
############################

def constant_rotate():

    # As soon as a obstacle is sensed by laser, this function is used to rotate...
    #... and align sideways with the obstacle.
    global twist_msg, regions, safe_dist, state_description, space_description

    if regions['left'] >= regions['right']:
        space_description=1
    
    elif regions['left'] < regions['right']:
        space_description=0

    # If there is more space on left, then rotate left.    
    if regions['left'] >= regions['right']:
        twist_msg.angular.z = 0.25
        twist_msg.linear.x = 0
        if (0.25*safe_dist) < regions['right'] < safe_dist and regions ['front'] > safe_dist :
            change_state(4)
        # If right region is more than 0.18 mtr and less than 0.75 mtr then it moves to 
        #state 4 which drives it straight
        # Also keep in mind that there should be nothing in front otherwise it will flip b/w 
        # state 3 and 4. And will ultimately ram into wall when at a corner
        # Added this 'front' condition during experimenting (when in a L shape corner, it rammed into wall)

    #Else rotate right and align with obstacle on left side
    elif regions['left'] < regions['right']: 
        twist_msg.angular.z = -0.25
        twist_msg.linear.x = 0
        if (0.25*safe_dist) < regions['left'] < safe_dist and regions ['front'] > safe_dist:
            change_state(4)

    
############################
##      Constant Ahead    ##
############################

def constant_ahead():

    # Once it has aligned with obstacle sideways, Robot will move alongside the obstacle...
    #... until 'state_description ==1'
    global state_description, regions, twist_msg, safe_dist, s
    twist_msg.angular.z = 0.05 if space_description==0 else -0.05
    twist_msg.linear.x = 0.3
    # Note that angular velocity is not zero, a small value is given so that
    # robot align itself with the wall always. If this is not given then robot was at many times
    # moving in a loop (wall on left > away from left > reorient (U-turn) > wall on right > away)


    # If something comes in front then change back to state 3
    if regions['front'] < safe_dist:
        change_state(3)

############################
##     Reached this Pt    ##
############################

def reached_this_point():
    # To mark that this target has been reached and it should move onto next
    global twist_msg, all_targets, err_posn
    twist_msg.linear.x = 0 #Stop Robot
    twist_msg.angular.z = 0
    change_state(0) #To reset the state of machine
    rospy.loginfo('Soldier acquired this target. Moving for next !!')
    rospy.loginfo('Positional Accuracy = {} mtrs'.format(err_posn))
    rospy.loginfo('{} more targets to go'.format(len(all_targets)))
    rospy.loginfo('######################')
    
    #set next target

############################
## Bypass-Target function ##
############################

def cancel_this_target(msg):
    #Function gets string as action server and skips the target is string == 'pass'
    if msg.data == 'pass':
        if all_targets!= [] : 
            rospy.loginfo('this Target was skipped by user')
            rospy.loginfo('######################')
            set_target()
            change_state(0)

        else: 
            rospy.loginfo('This is the last target which user skipped')
            rospy.loginfo('######################')
            quit()

    else:
        rospy.loginfo('Type -pass- if you want to skip this target')
    

############################
##     Main Function      ##
############################

def main():
    global all, pub, twist_msg, state_description, switch, goal_subs_count, action_server
    global curr_posn_y, curr_posn_x, yaw

    rospy.init_node('drive_sensibly')
    time.sleep(2)

    #Make a subscriber for Laser, but this is not needed in this code  
    #Subscriber that stores all target points
    rospy.Subscriber('/goals', PointArray, get_target)
    rospy.wait_for_message('/goals', PointArray) 

    while not rospy.is_shutdown():
        #Make a subscriber for Model state, to get posn of Robot
        rospy.Subscriber('/gazebo/model_states', ModelStates, get_current_posn)
        rospy.wait_for_message('/gazebo/model_states', ModelStates)
        
        #Make a publisher for cmd_vel topic
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        #Make a subscriber for Laser
        rospy.Subscriber('/scan', LaserScan, get_laser_scan)
        rospy.wait_for_message('/scan', LaserScan)
        
        #Define the Service Server that takes message type string...
        #... and callback function skips the targets and calls func to set a new target.
            
        rospy.Service("cancel_server", StringServiceMsg , cancel_this_target)
        
        #Run this loop until target point list is empty
        while all_targets != []:

            set_target()
            twist_msg=Twist() #Define a variable that equals Twist msg from cmd_vel topic
            
            # This variable is used to detect if the robot is stuck at the same position
            # It checks the displacement and orientation of the robot
            time_window = 0

            while True: 
                
                #Count Time instances
                if 0 <= time_window <= 0.025:
                    look_t0_x= curr_posn_x #Register Current posn at t = 0 sec
                    look_t0_y= curr_posn_y
                    yaw_t0 = yaw
        
                time_window = time_window + 0.02

                #See state graph for understanding this While loop

                if curr_state == 0:
                    fix_yaw(target_now)
                    pub.publish(twist_msg)
                
                elif curr_state == 1:
                    move_ahead(target_now)
                    pub.publish(twist_msg)

                elif curr_state == 2:
                    reached_this_point()
                    pub.publish(twist_msg)
                    break
                
                elif curr_state == 3:
                    constant_rotate()
                    pub.publish(twist_msg)

                elif curr_state == 4:
                    constant_ahead()
                    pub.publish(twist_msg)
                
                # Setting Time window of 10 secs
                if 9.9 <= time_window <= 10.1  :
                    look_t5_x= curr_posn_x
                    look_t5_y= curr_posn_y
                    yaw_t5 = yaw

                    move_in_5_sec = math.sqrt(pow((look_t5_x - look_t0_x),2) + pow((look_t5_y - look_t0_y),2))
                    threshold_move= 0.15
                    if move_in_5_sec < threshold_move and abs(yaw_t5 - yaw_t0) < threshold_move :
                        #rotate constantly and change state(0)
                        twist_msg.angular.z = 0.2 
                        pub.publish(twist_msg)
                        time.sleep(5)
                        rospy.loginfo('I was stuck, so I repositioned myself')
                        change_state(0)
                    
                    #empty the bucket
                    time_window = 0

                time.sleep(0.02)
                rospy.logdebug(state_description)

        rospy.sleep(2)
        rospy.loginfo('All targets acquired. Robot is tired now, need some rest !!')
        quit()
    

if __name__ == '__main__':
    main()

#<include file="$(find turtlebot3_gazebo)/launch/turtlebot3_empty_world.launch"/>

        
    
