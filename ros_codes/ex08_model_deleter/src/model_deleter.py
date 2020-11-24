#! /usr/bin/env python
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest                     
# Import the service message
import sys
import rospy

rospy.init_node('service_client')                                               
# Initialise a ROS node with the name service_client

#Overview: call service --> create client variable --> create object variable -->...
#... assign object name --> call client variable with object variable as argument

rospy.wait_for_service('/gazebo/delete_model')                                  
# Wait for the service client /gazebo/delete_model
delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)  
# Create the service client
kk = DeleteModelRequest()                                                       
# Create an object of type DeleteModelRequest
kk.model_name = "right_wall"                                                    
# Fill the variable model_name of this object with the desired value
result = delete_model_service(kk)                                               
# Send through the connection the name of the object to the service node

print result  
