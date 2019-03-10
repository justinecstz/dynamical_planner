#!/usr/bin/env python
import roslib
import rospy
import math
import tf

from std_msgs.msg import String
from scipy import *

if __name__ = '__main__':
    listener = tf.TransformListener() #appel du constructeur
    ids = [0,1,2,3,4,5,6,7] #trouver comment récupérer les ids des markers pour que le code puisse être                     appliqué avec d'autres objets
    visibleMarkers = []
    i = 0
    while i < len(ids):
        fullId = """{type}{index}""".format(type = "marker/",index = ids[i])
        i = i+1
        bool = listener.canTransform("camera/1",fullId,rospy.Time())
        if bool == 1: #le marker est visible
            visibleMarkers.append(ids[i])

print(visibleMarkers)

