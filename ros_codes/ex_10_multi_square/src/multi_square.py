#! /usr/bin/env python

import rospy
from ex10_my_srv_msg.srv import MyServiceMsg, MyServiceMsgResponse # you import the service message python class
from geometry_msgs.msg import Twist
import math


#Function to publish velocity and move robot
def move_robot(rad,rep):
    rate=rospy.Rate(0.1) # Set a publish rate of 10 seconds
    wait=rospy.Rate(0.5) # Set a publish rate of 1 Hz
    pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10) # Create a Publisher object, that will publish
    t=Twist() #Create the t variable
    wait.sleep()

    for i in range(4*rep):

        #turn 90 degree and stop
        t.angular.z=0.157
        pub.publish(t)
        rate.sleep()
        t.angular.z=0
        pub.publish(t)
        wait.sleep()

        #move straight and stop
        t.linear.x=rad/10
        pub.publish(t)
        rate.sleep()
        t.linear.x=0
        pub.publish(t)
        wait.sleep()

        i=i+1

#Define function and task to be done when service is called
def my_callback(request):
    
    rep=request.repetitions
    rad=request.radius

    move_robot(rad,rep)
    
    return MyServiceMsgResponse() # the service Response class, in this case EmptyResponse

#Initiate the node
rospy.init_node('multi_square')

#Creates with service with name ,message and function
 
multi_square_service = rospy.Service('multi_square_service', MyServiceMsg, my_callback) # create the Service
rospy.spin() # mantain the service open.
