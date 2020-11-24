#!/usr/bin/env python

##########################################
##       Created by: SAHIL ARORA        ## 
##  HOCHSCHULE- RAVENSBURG WEINGARTEN   ##
##          Date: 21 Juli 2020          ##
##########################################

import rospy
import actionlib
import math
import numpy
import time
import operator as os

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from goal_publisher.msg import GoalPoint, PointArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf import transformations

#Intializations of variables
goal_subs_count=0
later_goal = []
reward_gain = 0
retry = 0

# This variable is used to detect if the robot is stuck at the same position
# It checks the displacement and orientation of the robot
time_window = 0


############################
##    Get All Targets     ##
############################
# reused from mini_proj

def get_target(msg):
    #Takes callback from /goal_publisher package (/goals publisher) and gives target points.
    #Takes 'PointArray' msg which has 'goals' message 
    global all_targets, goal_subs_count

    #Defined to take the msg only once and store it in list all_targets
    if goal_subs_count == 0:
        all_targets=msg.goals
        rospy.loginfo('Targets to Cover by WALLE: {}'.format(len(all_targets)))
        goal_subs_count = goal_subs_count + 1


############################
##     Get Laser Scan     ##
############################

def get_laser_scan(msg):
    global regions, safe_dist, state_description, obs_space, free_space_name, free_space_val
  
    regions = { 'right': min(min(msg.ranges[252:287]), 10), 
    'f_right': min(min(msg.ranges[288:340]), 10), 
    'front': min(min(msg.ranges[0:17]), 10,min(msg.ranges[341:359])), 
    'f_left': min(min(msg.ranges[18:70]), 10),
    'left': min(min(msg.ranges[71:106]), 10) } 
    #180/5= 36 each
    
    safe_dist=0.75 #Safe distance to Obstacles in metres

    # Used when the robot is stuck to check which is the most free space...
    # ...Divided into F,L,R,B with 8 beams in each sector

    obs_space = { 'right': min(min(msg.ranges[266:273]), 10), 
    'back': min(min(msg.ranges[176:183]), 10), 
    'front': min(min(msg.ranges[0:3]), 10,min(msg.ranges[356:359])), 
    'left': min(min(msg.ranges[86:93]), 10) } 

    # Sorting the above variable in order
    free_space_name = max(obs_space.iteritems(), key= os.itemgetter(1))[0]
    free_space_val = max(obs_space.iteritems(), key= os.itemgetter(1))[1]

############################
##  Set Profitable Target ##
############################

def set_target():

    # Callback function that calculates distance to all pending targets...
    #... and applies a weighted function reward/(dist). Then it takes the highest
    #...one and removes the chosen target from the all_targets list nce it is completed.

    global profit_target, all_targets, target_now
    profit_target=[None]*len(all_targets) #Empty List of same length as all_targets list

    # Iteratively calculate for all points
    for i in range(len(all_targets)):
        dist = math.sqrt(pow((all_targets[i].x-current_x_pos),2) + pow((all_targets[i].y-current_x_pos),2))
        profit_target[i] = (all_targets[i].reward)/dist
        i=i+1
    
    all_targets = [all_targets for _,all_targets in sorted(zip(profit_target,all_targets), reverse= True)]
    # Sorting all targets based on distance
    
    target_now = all_targets[0] #Target Chosen

    # Print-outs
    rospy.loginfo('Target set. Soldier approaching target !!')
    rospy.loginfo('Most profitable : t_x={} and t_y={}'.format(target_now.x,target_now.y))


############################
##  Get Current Position  ##
############################

def get_current_position(pos):
    #Takes callback from /amcl_pose topic and gives position and orientation.
    global current_x_pos, current_y_pos, current_theta ,err_posn
    
    # Taken from rosmsg type of amcl. See comments of issue #7
    current_x_pos = pos.pose.pose.position.x
    current_y_pos = pos.pose.pose.position.y
    
    #Define orientation of Robot.
    rot_q = pos.pose.pose.orientation
    quaternions = ( rot_q.x , rot_q.y , rot_q.z , rot_q.w )

    #ROS gives output in quaternion system. need to be converted to Euler angles.
    euler = transformations.euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
    current_theta = euler[2] #Take the z component



############################
##       360 Origin      ##
############################
 
# This function is executed upon startup if the program. It rotates the robot for a full...
#... degree so that amcl particle particle is converged. If this is not done it poses problem...
# ... when robot is spawbed very near to a goal point.

def origin():
    global twist_msg

    twist_msg.angular.z = 0.6283
    

############################
##      Unstuck Robot     ##
############################

# This function is called when robot is stuck due to DWA planner failure. It identifies..
# ..the area with max space and then reoriens into that direction so that future goals are..
# ..traversed easliy.

def unstuck():
    global twist_msg

    # Do a linear movement when there is space in front/ back.
    if free_space_name == 'back' or free_space_name == 'front':
        twist_msg.linear.x = -0.02 if free_space_name == 'back' else 0.02 
        twist_msg.angular.z = 0
        rospy.logwarn('Backing off !')

    # Do a angular movement when there is space in left/ right.
    if free_space_name == 'left' or free_space_name == 'right':
        twist_msg.angular.z = -0.31416 if free_space_name == 'right' else 0.31416
        twist_msg.linear.x = 0
        rospy.logwarn('Re-orienting !')

    # Note: +ve angular vel means anticlockwise movement

############################
##     Main Function      ##
############################

def main():
    global all, goal_subs_count, action_server , err_posn, later_goal, time_window, twist_msg
    global current_x_pos, current_y_pos, current_theta, all_targets, target_now, reward_gain, retry

    rospy.init_node('prj_walle_src')
    time.sleep(0.5)

    #Subscriber that stores all target points.
    rospy.Subscriber('/goals', PointArray, get_target)
    rospy.wait_for_message('/goals', PointArray)

    while not rospy.is_shutdown():
        
        # Subscribe to /amcl_pose topic to localize the turtlebot in the given map
        localize = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped , get_current_position)

        # Create an action client called "move_base" with action message "MoveBaseAction"
        move_clnt = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # Waits until the action server has started up and started listening for goals.
        move_clnt.wait_for_server()

        #Make a publisher for cmd_vel topic
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        twist_msg=Twist() #Define a variable that equals Twist msg from cmd_vel topic

        #Make a subscriber for Laser
        rospy.Subscriber('scan', LaserScan, get_laser_scan)
        rospy.wait_for_message('scan', LaserScan)

        # Creates a new message variable 't' of type MoveBaseGoal
        t = MoveBaseGoal()

        # Call origin() func to converge particle cloud on start-up
        origin()
        time.sleep(2)
        pub.publish(twist_msg)
        time.sleep(10)

        while all_targets != []:
    
            #target_now = all_targets[0]
            set_target()

            # Give the details of the goal point as move_base_msgs/MoveBaseActionGoal 
            t.target_pose.header.frame_id = "map" # Move to specified point in "map" coordinate frame 
            t.target_pose.header.stamp = rospy.Time.now()
            t.target_pose.pose.position.x = target_now.x
            t.target_pose.pose.position.y = target_now.y
            t.target_pose.pose.orientation.w = 1

            # Send goal information to the client made above
            move_clnt.send_goal(t)
            
            while True:

                #Count Time instances
                if 0 <= time_window <= 0.025:
                    look_t0_x= current_x_pos # Register Current posn at t = 0 sec
                    look_t0_y= current_y_pos
                    yaw_t0 = current_theta # Register Orientation posn at t = 0 sec

                # Recursively increment time window
                time_window = time_window + 0.02

                # Positional Accuracy
                err_posn = math.sqrt(pow((target_now.x-current_x_pos),2) + pow((target_now.y-current_y_pos),2))

                # FINAL STATE 1 : REACHED
                if move_clnt.get_state() == 3:

                    rospy.loginfo('Reached given point !!')
                    rospy.loginfo('Positional Accuracy = {} mm'.format(err_posn * 1000))
                    
                    # Cumulative calculation.
                    reward_gain = reward_gain + target_now.reward
                    
                    rospy.loginfo('WALLE says: Hey judges ! I am pausing for 1.0 secs. Please note my reward points.')
                    time.sleep(1.25) # Pre-requisite in problem statement

                    rospy.loginfo('Cumulative reward gain: {}'.format(reward_gain))
                    rospy.loginfo('######################')

                    # Remove the acheived goal point from targets list
                    all_targets = all_targets[1:]
                    
                    # Put here to solve a problem, see Readme ofr details about the problem
                    time_window = 0
                    break

                # FINAL STATE 2 : ABORTED
                if move_clnt.get_state() == 4:
                    
                    # If aborted by move_base, goal is saved for later by manual drive.
                    rospy.logwarn('Goal saved for later (Scenario: Aborted). Moving to next target.')
                    later_goal.append(target_now)
                    all_targets = all_targets[1:] # Remove goal from main list
                        
                    time_window = 0 # Empty Bucket
                    move_clnt.cancel_goal() # Stop move base action server
                    
                    unstuck() # Call function to decide where to move
                    time.sleep(2)
                    pub.publish(twist_msg) # Publish msg
                    time.sleep(5)

                    rospy.loginfo('######################')
                    break
                
                # FINAL STATE 3 : STUCK DUE TO DWA FAILURE 

                # This block check the diplacement of the robot ater a timw eindow of 10 secs..
                # ..If displacement < threshold then the goal will be forfeited for the moment..
                # .. and can be tried later

                if 9.9 <= time_window <= 10.2  :
                    look_t5_x= current_x_pos # Register Current posn at t = 10 sec
                    look_t5_y= current_y_pos
                    yaw_t5 = current_theta

                    # Compute displacement
                    move_in_5_sec = math.sqrt(pow((look_t5_x - look_t0_x),2) + pow((look_t5_y - look_t0_y),2))
                    threshold_move= 0.2

                    # Compare with threshold
                    if move_in_5_sec < threshold_move:
                        
                        rospy.logwarn('Goal saved for later (Scenario: Window). Moving to next target.')
                        later_goal.append(target_now)
                        all_targets = all_targets[1:] # Remove goal from main list
                        
                        time_window = 0 # Empty Bucket
                        move_clnt.cancel_goal() # Stop move base action server
                        
                        unstuck() # Call function to decide where to move
                        time.sleep(2)
                        pub.publish(twist_msg) # Publish msg
                        time.sleep(5)

                        rospy.loginfo('######################')
                        break
                    
                    # Empty the bucket
                    time_window = 0

                # RETRY for all the leftover points with move_base 
                # Some points are not achieved because the robot got stuck or some other bot was blocking..
                #.. So goal points, which were saved in later_goal list then, are appended to all_taret list iteratively.
                # This is done a max of 4 times.

                if len(all_targets) == 1:
                    # if only 1 goal is left, only then append
                    if retry < 4:

                        rospy.logwarn('Points for retry after current target: {}'.format(len(later_goal)))
                        rospy.logwarn('Retry Count : {}'.format(retry+1))

                        # Append each point iteratively
                        for i in range(len(later_goal)):
                            all_targets.append(later_goal[i])

                        retry += 1
                        later_goal = [] # Empty the goal list

                time.sleep(0.02)


        # Once the robot has finished with the work, it is instructed to go to a home position..
        #.. in our case it is set to 0,0
        rospy.loginfo('I am tired. Going home.........')
        t.target_pose.header.frame_id = "map" # Move to specified point in "map" coordinate frame 
        t.target_pose.header.stamp = rospy.Time.now()
        t.target_pose.pose.position.x = 0
        t.target_pose.pose.position.y = 0
        t.target_pose.pose.orientation.w = 1

        # Send goal
        move_clnt.send_goal(t)
        move_clnt.wait_for_result()
        if move_clnt.get_state() == 3:
            rospy.loginfo('Shutting down......... Please kill the node manually.')

        if move_clnt.get_state() == 4:
            rospy.loginfo('HELP......... Somebody please manually replace me to home.')

        time.sleep(0.05)
        


if __name__ == '__main__':
    main()