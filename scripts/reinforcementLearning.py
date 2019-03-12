#!/usr/bin/env python
import roslib
import rospy
import math
import tf
from std_msgs.msg import String
import numpy as np


if __name__ == '__main__' :
   rospy.init_node('reinforcementLearning')

   #definition des matrices de proba pour les 12 states et 6 actions
   #probaMatrices = [np.zeros((12,12)), np.zeros((12,12)), np.zeros((12,12)), np.zeros((12,12)), np.zeros((12,12)), np.zeros((12,12))]
   #action 1: pick(1)
   #probaMatrices[0][(0, 11)], probaMatrices[0][(1, 10)], probaMatrices[0][(2, 7)], probaMatrices[0][(3, 8)] = 1, 1, 1, 1
   #probaMatrices[0][(4, 4)], probaMatrices[0][(5, 5)], probaMatrices[0][(6, 6)], probaMatrices[0][(7, 7)],  = 1, 1, 1, 1
   #probaMatrices[0][(8, 8)], probaMatrices[0][(9, 9)], probaMatrices[0][(10, 10)], probaMatrices[0][(11, 11)],  = 1, 1, 1, 1


