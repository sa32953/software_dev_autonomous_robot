#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse # you import the service message python class
from geometry_msgs.msg import Twist


#Define function and task to be done when service is called
def my_callback(request):
    rate=rospy.Rate(0.1) # Set a publish rate of 2 Hz
    pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1) # Create a Publisher object, that will publish
    t=Twist() #Create the t variable

    #move straight and stop
    t.linear.x=0.0157
    pub.publish(t)
    rate.sleep()
    t.linear.x=0
    pub.publish(t)
    rate.sleep()

    #turn 90 degree
    t.angular.z=0.157
    pub.publish(t)
    rate.sleep()
    t.angular.z=0
    pub.publish(t)
    rate.sleep()
    
    return EmptyResponse() # the service Response class, in this case EmptyResponse

#Initiate the node
rospy.init_node('square_service_client')

#Creates with service with name ,message and function
my_service = rospy.Service('/square_service', Empty , my_callback) # create the Service
rospy.spin() # mantain the service open.