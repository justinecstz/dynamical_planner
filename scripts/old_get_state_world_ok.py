#!/usr/bin/env python
import roslib
import rospy
import math
import tf
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
import numpy as np

oldState = "" 

if __name__ == '__main__' :
   rospy.init_node('getStateWolrd')
   listenerMarkers = tf.TransformListener(True, rospy.Duration(5)) #appel du constructeur
   listenerCubes = tf.TransformListener(True, rospy.Duration(5)) #appel du constructeur
   rospy.sleep(1.5) #laisser le temps a tf de s'initialiser
   ids = [0, 1, 3] #trouver comment recuperer les ids des markers pour que le code puisse etre applique avec d'autres objets
   human = 0
   lettersId = ["N", "N", [], "N", [], [], "N", "N"] #pour markers 0,1,3,6,7
   rate = rospy.Rate(1.0) #2 Hz, donc publication deux fois par seconde

   global oldState

   # listener.waitForTransform("marker/0","camera/1",rospy.Time.now(), rospy.Duration(0.01))
   # transform = listener.lookupTransform("marker/0","camera/1",rospy.Time(0))
   # print(transform)
   # listener.waitForTransform("marker/1","camera/1",rospy.Time.now(), rospy.Duration(0.01))
   # transform = listener.lookupTransform("marker/1","camera/1",rospy.Time(0))
   # print(transform)
   # listener.waitForTransform("marker/3","camera/1",rospy.Time.now(), rospy.Duration(0.01))
   # transform = listener.lookupTransform("marker/3","camera/1",rospy.Time(0))
   # print(transform)

   #getStateWorld
   while not rospy.is_shutdown() :
      state = ""
      visibleMarkers = []
      visibleCubes = []
      xPositionMarkers = []
      xPositionCubes = []
      yPositionMarkers = []
      yPositionCubes = []
      zPositionMarkers = []
      zPositionCubes = []
      xPosition = []
      itemsLeftToRight = []
      index = []
      visible = []
      for id_marker in ids :
         fullId = "marker/" + str(id_marker)
         # if listenerMarkers.canTransform(fullId, "camera/1", rospy.Time.now() - rospy.Duration(1.0)) :
         if listenerMarkers.canTransform("camera/1", fullId, rospy.Time.now() - rospy.Duration(1.0)) :
            visibleMarkers.append(id_marker)
            # (position,quaternion) = listenerMarkers.lookupTransform(fullId,"camera/1",rospy.Time(0))
            (position,quaternion) = listenerMarkers.lookupTransform("camera/1", fullId, rospy.Time(0))
            #print(position)
            xPositionMarkers.append(position[0])
            yPositionMarkers.append(position[1])
            zPositionMarkers.append(position[2])

      if listenerCubes.canTransform("camera/1","green",rospy.Time.now() - rospy.Duration(1.0)) :
         visibleCubes.append("G")
         (position,quaternion) = listenerCubes.lookupTransform("camera/1", "green", rospy.Time(0))
         xPositionCubes.append(position[0]*0.0006-0.218) 
         yPositionCubes.append(position[1]*0.0007-0.1972)
         zPositionCubes.append(position[2])
         #print(position)
         # print(xPositionCubes)
         # print(yPositionCubes)
         # print(zPositionCubes)

      if listenerCubes.canTransform("camera/1", "blue", rospy.Time.now() - rospy.Duration(1.0)) :
         visibleCubes.append("B")
         (position,quaternion) = listenerCubes.lookupTransform("camera/1","blue", rospy.Time(0))
         xPositionCubes.append(position[0]*0.0006-0.218)
         yPositionCubes.append(position[1]*0.0007-0.1972)
         zPositionCubes.append(position[2])


      visibleItems = visibleMarkers + visibleCubes
      xPosition = xPositionMarkers + xPositionCubes
      yPosition = yPositionMarkers + yPositionCubes
      zPosition = zPositionMarkers + zPositionCubes

      length = np.arange(len(visibleItems))
      for k in length :
         visible.append([visibleItems[k],xPosition[k],yPosition[k],zPosition[k]])

      # x = np.array(xPosition) 
      # x = np.argsort(x)
      y = np.array(yPosition)
      y = np.argsort(y)

      totalItems = len(visibleMarkers) + len(visibleCubes)

      i = 0

      for k in np.arange(totalItems) :
         # while xPosition[x[k]] != visible[i][1] :
         while yPosition[y[k]] != visible[i][2] :
            i = i + 1
         if visible[i][0] != "G" and visible[i][0] != "B" :
            state = state + "N"
         else :
            state = state + visible[i][0]
         i = 0 

      pub = rospy.Publisher('currentState', String, queue_size = 10)
      pubHandover = rospy.Publisher('handover', String, queue_size = 10)

      if listenerMarkers.canTransform("camera/1", "marker/7", rospy.Time.now() - rospy.Duration(1.0)) :
         human = 1
      else :
         human = 0

      if listenerMarkers.canTransform("camera/1", "marker/6", rospy.Time.now() - rospy.Duration(1.0)) :
         pubHandover.publish("handover")
      else :
         pubHandover.publish("no handover")

      if "B" in state and "G" in state:
         stateFull = state + "N"
      elif "B" in state and not "G" in state:
         if human == 1 and oldState[-1] == "G":
            stateFull = state + "G"
         elif human == 1 and oldState[-1] == "N":
            stateFull = state + "N"
         elif human == 0:
            stateFull = state + "G"
      elif "G" in state and not "B" in state:
         if human == 1 and oldState[-1] == "B":
            stateFull = state + "B"
         elif human == 1 and oldState[-1] == "N":
            stateFull = state + "N"
         elif human == 0 :
            stateFull = state + "B"
      elif not "B" in state and not "G" in state:
         if human == 1 :
            for i in np.arange(len(oldState)):
               if oldState[i] != "N":
                  stateFull = state + oldState[i]     

      oldState = stateFull 
      # print(state)
      # if "B" in state and "G" in state :
      #    state = state + "N"
      # elif "B" in state and not "G" in state :
      #    state = state + "G"
      # elif "G" in state and not "B" in state :
      #    state = state + "B"
      if len(stateFull) == 4 :
         rospy.loginfo(stateFull)
         pub.publish(stateFull)
         rate.sleep()
#print(state)