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

def demoLaunch() :
	nbDemos = input("Enter the number of demonstration sequences")
	demos = []
	currentDemo = []
	i = 0
	key = 0
	while i < nbDemos :
	   while key != "q"
          state = rospy.Subscriber('stateTopic', String,callback)
          currentDemo.append(state)
          input("Once next state is ready, press Enter. To jump to next demo, press q")
       i = i + 1
       


if __name__ == '__main__' :
   rospy.init_node('planner')
   rospy.sleep(1.5) #laisser le temps a tf de s'initialiser
   statesDemo = []
   rewards = np.zeros((12, 6))
   markovMap = [[],[],[],[],[],[],
      			[],[],[],[],[],[]]

   #test = rospy.Subscriber('stateTopic', String,callback)
   
   #demo
   demoLaunch()
   # loopSteps = np.arange(len(statesDemo) - 1) #len = 13
   # markovMap = np.zeros((len(statesDemo) - 1, 5)) #enlever 1 car le premier etat de demo se repete
   # i = 0
   # for i in loopSteps : #de 0 a 11
   #    if i == 0 : 
   # 	     currentState = statesDemo[i]
   #       markovMap((i, 0)) = currentState
   #       markovMap((i, 1)) = statesDemo[- 2]
   #       markovMap((i, 2)) = statesDemo[i + 1]
   #       #actions autorisees
   #       for j in np.arange(3) :
   #          if markovMap((i, 0))(j) == "B" :

   #    else :
   #    	 currentState = statesDemo[i]
   #       markovMap((i, 0)) = currentState
   #       markovMap((i, 1)) = statesDemo[i - 1]
   #       markovMap((i, 2)) = statesDemo[i + 1]


         





 

