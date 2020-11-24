#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf import transformations
from tf.transformations import euler_from_quaternion

rospy.init_node('first')
rate=rospy.Rate(2) # Set a publish rate of 2 Hz

goal_list = [ [2.08,3.2]]
#print(goal_list[0])
rate = rospy.Rate(2)

############################
##  Get Current Position  ##
############################

def callback_position(pos):
    global current_x_pos
    global current_y_pos
    global current_theta
    current_x_pos = pos.pose.pose.position.x
    current_y_pos = pos.pose.pose.position.y
    rot_q=pos.pose.pose.orientation
    (roll,pitch,current_theta)=euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])



# Subscribe to /amcl_pose topic to localize the turtlebot in the given map
sub_loc = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped , callback_position)

# Create an action client called "move_base" with action message "MoveBaseAction"
client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
# Waits until the action server has started up and started listening for goals.
client.wait_for_server()

# Creates a new message t of type MoveBaseGoal
t = MoveBaseGoal()
print('1')
#Move to points one by one
while not rospy.is_shutdown() :

    print('2')
    while goal_list != []:
        des = goal_list[0]

        # Move to des point in "map" coordinate frame 
        t.target_pose.header.frame_id = "map"
        t.target_pose.header.stamp = rospy.Time.now()
        t.target_pose.pose.position.x = goal_list[0][0]
        t.target_pose.pose.position.y = goal_list[0][1]
        t.target_pose.pose.orientation.w = 1
        client.send_goal(t)
        wait = client.wait_for_result()

        if client.get_state() == 3:
            print('reached')
            #rospy.sleep(rate)
            goal_list = goal_list [1:]

    rospy.sleep(rate)

# Move to x,y point in "map" coordinate frame 
#t.target_pose.header.frame_id = "map"
#t.target_pose.header.stamp = rospy.Time.now()
#t.target_pose.pose.position.x = 0.5
#t.target_pose.pose.position.y = -0.5

# No rotation of the mobile base frame w.r.t. map frame (given in Quaternions)
#t.target_pose.pose.orientation.w = 1

#client.send_goal(t)
#wait = client.wait_for_result()


