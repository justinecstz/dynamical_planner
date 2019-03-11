#!/usr/bin/env python
import roslib
import rospy
import math
import tf
from std_msgs.msg import String
import numpy as np

def callback(data) :
	rospy.loginfo(data.data)
	print(data)

if __name__ == '__main__' :
   rospy.init_node('planner')
   rospy.sleep(1.5) #laisser le temps a tf de s'initialiser

   #subscriber
   rospy.Subscriber('stateTopic', String,callback)
