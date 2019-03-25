#!/usr/bin/env python
import roslib
import rospy
import math
import tf
from std_msgs.msg import String
import numpy as np


#def callback(data) :
	#a terme, predire l'action suivant le state recu 
	#rospy.loginfo(data.data)
	#return data

# def demoLaunch() :
#    nbDemos = input("Enter the number of sequences of demonstration")
#    demos = []
#    currentDemo = []
#    i = 0
#    key = ""
#    while i < nbDemos :
#       while key != "q" :
#          state = rospy.Subscriber('stateTopic', String,callback)
#          currentDemo.append(state)
#          key = input("Once next state is ready, press n. To jump to next demo, press q")
#       i = i + 1

if __name__ == '__main__' :
  rospy.init_node('planner')
  #rospy.sleep(1.5) #laisser le temps a tf de s'initialiser
  #demoLaunch()
  #subscriber
  rate = rospy.Rate(1.0)
  #demoLaunch()
  #print(demos)
  #rospy.Subscriber('stateTopic', String,callback)
  while not rospy.is_shutdown() :
    rospy.Subscriber('stateTopic', String, callback) #when a message is received, callback is invoked w/ message as 1st arg
    rate.sleep()
