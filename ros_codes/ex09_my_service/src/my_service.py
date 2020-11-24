#! /usr/bin/env python
import rospy
from std_srvs.srv import Empty, EmptyResponse # you import the service message python class

#Define function and task to be done when service is called
def my_callback(request):
    print "My callback has been called"
    return EmptyResponse() # the service Response class, in this case EmptyResponse

#Initiate the node
rospy.init_node('service_client')

#Creates with service with name ,message and function
my_service = rospy.Service('/my_service', Empty , my_callback) # create the Service
rospy.spin() # mantain the service open.
