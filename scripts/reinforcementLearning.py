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
   probaMatrices = [np.zeros((12,12)), np.zeros((12,12)), np.zeros((12,12)), np.zeros((12,12)), np.zeros((12,12)), np.zeros((12,12))]
   #action 1: pick(1)
   probaMatrices[0][(0, 11)], probaMatrices[0][(1, 10)], probaMatrices[0][(2, 7)], probaMatrices[0][(3, 8)] = 1, 1, 1, 1
   probaMatrices[0][(4, 4)], probaMatrices[0][(5, 5)], probaMatrices[0][(6, 6)], probaMatrices[0][(7, 7)],  = 1, 1, 1, 1
   probaMatrices[0][(8, 8)], probaMatrices[0][(9, 9)], probaMatrices[0][(10, 10)], probaMatrices[0][(11, 11)],  = 1, 1, 1, 1
   #action 2: pick(2)
   probaMatrices[1][(0, 0)], probaMatrices[1][(1, 6)], probaMatrices[1][(2, 9)], probaMatrices[1][(3, 3)] = 1, 1, 1, 1
   probaMatrices[1][(4, 11)], probaMatrices[1][(5, 8)], probaMatrices[1][(6, 6)], probaMatrices[1][(7, 7)],  = 1, 1, 1, 1
   probaMatrices[1][(8, 8)], probaMatrices[1][(9, 9)], probaMatrices[1][(10, 10)], probaMatrices[1][(11, 11)],  = 1, 1, 1, 1
   #action 3: pick(3)
   probaMatrices[2][(0, 6)], probaMatrices[2][(1, 1)], probaMatrices[2][(2, 2)], probaMatrices[2][(3, 9)] = 1, 1, 1, 1
   probaMatrices[2][(4, 7)], probaMatrices[2][(5, 10)], probaMatrices[2][(6, 6)], probaMatrices[2][(7, 7)],  = 1, 1, 1, 1
   probaMatrices[2][(8, 8)], probaMatrices[2][(9, 9)], probaMatrices[2][(10, 10)], probaMatrices[2][(11, 11)],  = 1, 1, 1, 1
   #action 4: place(1)
   probaMatrices[3][(0, 0)], probaMatrices[3][(1, 1)], probaMatrices[3][(2, 2)], probaMatrices[3][(3, 3)] = 1, 1, 1, 1
   probaMatrices[3][(4, 4)], probaMatrices[3][(5, 5)], probaMatrices[3][(6, 6)], probaMatrices[3][(7, 2)],  = 1, 1, 1, 1
   probaMatrices[3][(8, 3)], probaMatrices[3][(9, 9)], probaMatrices[3][(10, 1)], probaMatrices[3][(11, 0)],  = 1, 1, 1, 1
   #action 5: place(2)
   probaMatrices[4][(0, 0)], probaMatrices[4][(1, 1)], probaMatrices[4][(2, 2)], probaMatrices[4][(3, 3)] = 1, 1, 1, 1
   probaMatrices[4][(4, 4)], probaMatrices[4][(5, 5)], probaMatrices[4][(6, 1)], probaMatrices[4][(7, 7)],  = 1, 1, 1, 1
   probaMatrices[4][(8, 5)], probaMatrices[4][(9, 2)], probaMatrices[4][(10, 10)], probaMatrices[4][(11, 4)],  = 1, 1, 1, 1
   #action 6: place(3)
   probaMatrices[5][(0, 0)], probaMatrices[5][(1, 1)], probaMatrices[5][(2, 2)], probaMatrices[5][(3, 3)] = 1, 1, 1, 1
   probaMatrices[5][(4, 4)], probaMatrices[5][(5, 5)], probaMatrices[5][(6, 0)], probaMatrices[5][(7, 4)],  = 1, 1, 1, 1
   probaMatrices[5][(8, 8)], probaMatrices[5][(9, 3)], probaMatrices[5][(10, 5)], probaMatrices[5][(11, 11)],  = 1, 1, 1, 1

