#!/usr/bin/env python

import rospy
from ex06_age_msg.msg import Age
#imports msg from the package we created in prev task

rospy.init_node('use_msg')
pub=rospy.Publisher('/age_topic', Age, queue_size=1)
#creates variable that publishes <to this topic> <this type of msg>
a=Age()
#define variable that gets the data
#define arbitrary values
a.years=25
a.months=1
a.days=30

rate=rospy.Rate(2)

while not rospy.is_shutdown():
  pub.publish(a) #publish 'a' variable with properties defined in pub variable
  print(a) #print the variable on screen, can be taken through rostopic echo <topic name>
  rate.sleep()
