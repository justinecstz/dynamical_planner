#!/usr/bin/env python
import roslib
import rospy
import math
import tf
from std_msgs.msg import String
import numpy as np


if __name__ == '__main__' :
   rospy.init_node('tf_listener')
   listener = tf.TransformListener(True, rospy.Duration(5)) #appel du constructeur
   rospy.sleep(1.5) #laisser le temps a tf de s'initialiser
   ids = [0, 1, 2, 3, 4, 5, 6, 7] #trouver comment recuperer les ids des markers pour que le code puisse etre applique avec d'autres objets
   visibleMarkers = []
   xPosition = []
   markersLeftToRight = []
   lettersId = ["N", "N", [], "N", [], [], "B", "V"] #pour markers 0,1,3,6,7
   index = []
   state = ""
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
   for id_marker in ids :
      #fullId = """{type}{index}""".format(type = "marker/",index = ids[i])
      fullId = "marker/" + str(id_marker)
      if listener.canTransform(fullId, "camera/1", rospy.Time.now() - rospy.Duration(1.0)) :
         visibleMarkers.append(id_marker)
         (position,quaternion) = listener.lookupTransform(fullId,"camera/1",rospy.Time(0))
         xPosition.append(position[0])

   x = np.array(xPosition)
   #print(visibleMarkers)
   #print(x) 
   x = np.argsort(x)

   totalMarkers = len(visibleMarkers)
   i = 0

   while i < totalMarkers and not rospy.is_shutdown() :
      markersLeftToRight.append(visibleMarkers[x[i]])   #left to right relatif, attention a la direction des axes !
      index.append(i)
      state = state + str(lettersId[markersLeftToRight[i]])
      i = i + 1
      #print(markersLeftToRight)

    #identification des etats
    #print(state)

    #publisher
    pub = rospy.Publisher('stateTopic',String)
    #rate = rospy.Rate(10) #10 Hz
    rospy.loginfo(state)
    pub.publish(state)
    #rate.sleep()


   
