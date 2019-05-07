#!/usr/bin/env python
import roslib
import rospy
import math
import tf
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rospy.numpy_msg import numpy_msg
import numpy as np
#from threading import lock

markovMap = [[0, "GNBN", 0, 11, 2, 6], [1, "GBNN", 1, 6, 0, 10], [2, "BGNN", 0, 7, 1, 9], 
[3, "BNGN", 0, 8, 2, 9], [4, "NGBN", 1, 11, 2, 7], [5, "NBGN", 1, 8, 2, 10],
[6, "GNNB", 4, 1, 5, 0], [7, "NGNB", 3, 2, 5, 4], [8, "NNGB", 3, 3, 4, 5],
[9, "BNNG", 4, 2, 5, 3], [10, "NBNG", 3, 1, 5, 5], [11, "NNBG", 3, 0, 4, 4]]
#pour l'instant, la demo est pre-enregistree
#demos = [["NGBN", "NNBG", "GNBN", "GNNB", "GBNN"], ["NBGN", "NBNG", "GBNN"], ["BGNN", "BNNG", "BNGN", "NNGB", "NBGN", "NBNG", "GBNN"]]
demos = [["GBNN","GNNB","GNBN", "NNBG", "NGBN", "NGNB", "BGNN"],["GBNN","NBNG","NBGN","NNGB", "BNGN","BNNG","BGNN"]]
goal = demos[0][-1]
markovResults = []
oldAction = 10

# Simulation
# jointPositionsP1 = 1*np.array([-0.044405747733462106, 1.19814689840694254, 0.1520219301975148, -1.607911064798659, -0.07193516616802327, 0.3915826375219599, -1.568347410379195])
# jointPositionsP2 = 1*np.array([0.644405747733462106, 1.15814689840694254, 0.1520219301975148, -1.607911064798659, -0.07193516616802327, 0.3915826375219599, -1.568347410379195]) 
# jointPositionsP3 = 1*np.array([1.344405747733462106, 1.19814689840694254, 0.1520219301975148, -1.607911064798659, -0.07193516616802327, 0.3915826375219599, -1.568347410379195])
# jointPositionsHome = 1*np.array([0.644405747733462106, 0.69814689840694254, 0.1520219301975148, -1.607911064798659, -0.07193516616802327, 0.7915826375219599, -1.568347410379195])

# Real robot
jointPositionsP1 = 1*np.array([-0.4339979790845205, 1.3373235520341373, -0.1650918060481264, -0.39354840374588973, 0.16458329147693798, 1.4079477279662593, -1.570306351787338])
jointPositionsP2 = 1*np.array([-0.1526515215095898, 1.186118107041075, -0.4308292301986005, -0.757119167631625, 0.37156025122477176, 1.2697712925938716, -1.5704178650184049])
jointPositionsP3 = 1*np.array([0.008061009099187546, 1.140941414106668, -0.4140705499230182, -0.8524215590582609, 0.40469318154063466, 1.250843827247102, -1.5704030045215718])
jointPositionsHome = 1*np.array([-0.2561309292711379, 0.48341762688080325, -0.07932016919260708, -1.0536090715454243, -0.017271089745345972, 1.5119789493870104, -1.5704001283280475])

pick1 = [jointPositionsP1, jointPositionsHome]
pick2 = [jointPositionsP2, jointPositionsHome]
pick3 = [jointPositionsP3, jointPositionsHome]
place1 = [jointPositionsP1, jointPositionsHome]
place2 = [jointPositionsP2, jointPositionsHome]
place3 = [jointPositionsP3, jointPositionsHome]

class PdCtrl:
   def __init__(self):
      self.ctrl_freq = 200
      self.joint_names = ["iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"]
      self.joint_cmd_pub =  rospy.Publisher('/iiwa/PositionController/command', numpy_msg(Float64MultiArray),queue_size=10)

   def send_cmd(self,command,time_stamp): #command = Float64MultiArray
       next_joint_state = Float64MultiArray()

       next_joint_state.data = command 
       self.joint_cmd_pub.publish(next_joint_state)

   def lin_ds(self,current_joint_position, target_state):
       k = 0.9
       A = np.array([[k, 0, 0, 0, 0, 0, 0],
                     [0, k, 0, 0, 0, 0, 0],
                     [0, 0, k, 0, 0, 0, 0],
                     [0, 0, 0, k, 0, 0, 0],
                     [0, 0, 0, 0, k, 0, 0],
                     [0, 0, 0, 0, 0, k, 0],
                     [0, 0, 0, 0, 0, 0, k]])
      
       B = current_joint_position - target_state
       B = np.transpose(B)
       #next_joint_state = np.matmul(A, B) + current_joint_position
       next_joint_state = np.matmul(A, B) + target_state
       return next_joint_state

   def do_action(self,action) :
        rtol = 1e-4
        atol = 1e-4
        time = rospy.Time.now()
        r = rospy.Rate(controller.ctrl_freq)
        joint_states_msg = rospy.wait_for_message('iiwa/joint_states', numpy_msg(JointState))
        current_position = joint_states_msg.position
        if action == 0 :
            desired_joint_state_1 = pick1[0]
            desired_joint_state_2 = pick1[1]

        elif action == 1 :
            desired_joint_state_1 = pick2[0]
            desired_joint_state_2 = pick2[1]

        elif action == 2 :
            desired_joint_state_1 = pick3[0]
            desired_joint_state_2 = pick3[1]

        elif action == 3 :
            desired_joint_state_1 = place1[0]
            desired_joint_state_2 = place1[1]

        elif action == 4 :
            desired_joint_state_1 = place2[0]
            desired_joint_state_2 = place2[1]

        elif action == 5 :
            desired_joint_state_1 = place3[0]
            desired_joint_state_2 = place3[1]        

        while not np.allclose(desired_joint_state_1,current_position, rtol, atol) :
            # print(desired_joint_state)
            # print(current_position)
            next_joint_state = controller.lin_ds(current_position, desired_joint_state_1)
            controller.send_cmd(next_joint_state,time)
            r.sleep()
            joint_states_msg = rospy.wait_for_message('iiwa/joint_states', numpy_msg(JointState))
            current_position = joint_states_msg.position

        while not np.allclose(desired_joint_state_2,current_position, rtol, atol) :
            # print(desired_joint_state)
            # print(current_position)
            next_joint_state = controller.lin_ds(current_position, desired_joint_state_2)
            controller.send_cmd(next_joint_state,time)
            r.sleep()
            joint_states_msg = rospy.wait_for_message('iiwa/joint_states', numpy_msg(JointState))
            current_position = joint_states_msg.position


def reinforcementLearning(demos) :
#definition of probability matrices for the 12 states and 6 actions
   probaMatrices = [np.zeros((14, 14)), np.zeros((14, 14)), np.zeros((14, 14)), np.zeros((14, 14)), np.zeros((14, 14)), np.zeros((14, 14))]
   #action 0: pick(1)
   probaMatrices[0][(0, 11)], probaMatrices[0][(1, 10)], probaMatrices[0][(2, 7)], probaMatrices[0][(3, 8)] = 1, 1, 1, 1
   probaMatrices[0][(4, 4)], probaMatrices[0][(5, 5)], probaMatrices[0][(6, 6)], probaMatrices[0][(7, 7)],  = 1, 1, 1, 1
   probaMatrices[0][(8, 8)], probaMatrices[0][(9, 9)], probaMatrices[0][(10, 10)], probaMatrices[0][(11, 11)],  = 1, 1, 1, 1
   #probaMatrices[0][(12, 12)], probaMatrices[0][(13, 13)] = 1, 1
   #action 1: pick(2)
   probaMatrices[1][(0, 0)], probaMatrices[1][(1, 6)], probaMatrices[1][(2, 9)], probaMatrices[1][(3, 3)] = 1, 1, 1, 1
   probaMatrices[1][(4, 11)], probaMatrices[1][(5, 8)], probaMatrices[1][(6, 6)], probaMatrices[1][(7, 7)],  = 1, 1, 1, 1
   probaMatrices[1][(8, 8)], probaMatrices[1][(9, 9)], probaMatrices[1][(10, 10)], probaMatrices[1][(11, 11)],  = 1, 1, 1, 1
   #probaMatrices[1][(12, 12)], probaMatrices[1][(13, 13)] = 1, 1
   #action 2: pick(3)
   probaMatrices[2][(0, 6)], probaMatrices[2][(1, 1)], probaMatrices[2][(2, 2)], probaMatrices[2][(3, 9)] = 1, 1, 1, 1
   probaMatrices[2][(4, 7)], probaMatrices[2][(5, 10)], probaMatrices[2][(6, 6)], probaMatrices[2][(7, 7)],  = 1, 1, 1, 1
   probaMatrices[2][(8, 8)], probaMatrices[2][(9, 9)], probaMatrices[2][(10, 10)], probaMatrices[2][(11, 11)],  = 1, 1, 1, 1
   #probaMatrices[2][(12, 12)], probaMatrices[2][(13, 13)] = 1, 1
   #action 3: place(1)
   probaMatrices[3][(0, 0)], probaMatrices[3][(1, 1)], probaMatrices[3][(2, 2)], probaMatrices[3][(3, 3)] = 1, 1, 1, 1
   probaMatrices[3][(4, 4)], probaMatrices[3][(5, 5)], probaMatrices[3][(6, 6)], probaMatrices[3][(7, 2)],  = 1, 1, 1, 1
   probaMatrices[3][(8, 3)], probaMatrices[3][(9, 9)], probaMatrices[3][(10, 1)], probaMatrices[3][(11, 0)],  = 1, 1, 1, 1
   #probaMatrices[3][(12, 9)], probaMatrices[3][(13, 6)] = 1, 1
   #action 4: place(2)
   probaMatrices[4][(0, 0)], probaMatrices[4][(1, 1)], probaMatrices[4][(2, 2)], probaMatrices[4][(3, 3)] = 1, 1, 1, 1
   probaMatrices[4][(4, 4)], probaMatrices[4][(5, 5)], probaMatrices[4][(6, 1)], probaMatrices[4][(7, 7)],  = 1, 1, 1, 1
   probaMatrices[4][(8, 5)], probaMatrices[4][(9, 2)], probaMatrices[4][(10, 10)], probaMatrices[4][(11, 4)],  = 1, 1, 1, 1
   #probaMatrices[4][(12, 10)], probaMatrices[4][(13, 7)] = 1, 1
   #action 5: place(3)
   probaMatrices[5][(0, 0)], probaMatrices[5][(1, 1)], probaMatrices[5][(2, 2)], probaMatrices[5][(3, 3)] = 1, 1, 1, 1
   probaMatrices[5][(4, 4)], probaMatrices[5][(5, 5)], probaMatrices[5][(6, 0)], probaMatrices[5][(7, 4)],  = 1, 1, 1, 1
   probaMatrices[5][(8, 8)], probaMatrices[5][(9, 3)], probaMatrices[5][(10, 5)], probaMatrices[5][(11, 11)],  = 1, 1, 1, 1
   #probaMatrices[5][(12, 11)], probaMatrices[5][(13, 8)] = 1, 1

   #definition of the reward matrix
   reward = np.zeros((12, 6))

   #goal = BVNN
   goal = demos[0][len(demos[0])-1]
   for k in np.arange(len(markovMap)) :
      if goal in markovMap[k] :
         goalID = markovMap[k][0]
         break

   #conversion states' names into IDs
   indexDemo = np.arange(len(demos))
   demosID = []

   for i in indexDemo :
      indexCurrentDemo = np.arange(len(demos[i]))
      currentDemo = []
      for j in indexCurrentDemo :
         state = demos[i][j]

         for k in np.arange(len(markovMap)) :
            if state in markovMap[k] :
               currentDemo.append(markovMap[k][0])
               break

      demosID.append(currentDemo)
      #print(demosID)

   #construction reward matrix
   for i in indexDemo :
      #print(i)
      indexCurrentDemo = np.arange(len(demos[i]))
      for j in indexCurrentDemo :
         #print(j)
         stateID = demosID[i][j]
         #print(stateID)
         if stateID != goalID :
            if demosID[i][j + 1] == markovMap[stateID][3] :
               reward[stateID, markovMap[stateID][2]] = reward[stateID, markovMap[stateID][2]] + 2
            elif demosID[i][j + 1] == markovMap[stateID][5] :
               reward[stateID, markovMap[stateID][4]] = reward[stateID, markovMap[stateID][4]] + 2
   #print(reward)

   #construction of policy
   # policy = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
   # oldPolicy = [i * 2 for i in policy]
   policy = np.zeros((12,6))
   oldPolicy = np.ones((12,6))
   nbActions = 6
   U = np.zeros((12, 2))
   U[:,0], U[:,1] = 1, 2
   gamma = 0.8 
   epsilon = 0.1
   sumsOnActions = np.zeros((1, 6))
   sum1 = 0
   sum2 = 0

   #print(probaMatrices[5])
   #print(probaMatrices[5][1,1])

   while not np.allclose(oldPolicy, policy) :
      for i in np.arange(len(markovMap)) :
         while np.absolute(U[i, 0] - U[i, 1]) > epsilon :
            #print(np.absolute(U[i, 0] - U[i, 1]))
            U[i, 1] = U[i, 0]
            possibleActions = [markovMap[i][2], markovMap[i][4]]
            #print(possibleActions)
            for j in np.arange(nbActions) :
               for k in np.arange(len(markovMap)) : #s'
                  sum1 = sum1 + probaMatrices[j][i, k] * (reward[i, j] + gamma * U[k, 1])
                  #print(somme)
               sum2 = sum2 + sum1 * policy[i,j]
            U[i, 0] = sum2
            #print(U)
            sum1 = 0
            sum2 = 0
         oldPolicy[i,:] = policy[i,:]
         for j in np.arange(nbActions) :
            for k in np.arange(len(markovMap)) :
               sum1 = sum1 + probaMatrices[j][i, k] * (reward[i, j] + gamma * U[k, 1])
            sumsOnActions[0, j] = sum1
            sum1 = 0
         policy[i,np.argmax(sumsOnActions)] = 1 
         sumsOnActions = np.zeros((1, 6))
   #print(policy)

   markovResults = [policy, goalID]

   return markovResults

def handler_state_msgs(data) :
   #rospy.loginfo(data)
   #print(data.data)
   global oldAction

   state = data.data
   if state != goal :
      plannerDuo = planner(markovResults,state)
      optAction = plannerDuo[0]
      # time = rospy.Time.now()
      # r = rospy.Rate(controller.ctrl_freq)
      # pub.publish(optAction)
      # rospy.Subscriber('xPositions', Float32MultiArray, returnX)
      #rospy.Subscriber('yPositions', String, returnY)
      #rospy.Subscriber('zPositions', String, returnZ)

      if optAction == 0 : # pick(1)
         if oldAction != 0 :
            print("pick(1)")
            oldAction = 0
            controller.do_action(optAction)
            # next_joint_state = jointPositionsP1
            # controller.send_cmd(next_joint_state,time)
            # r.sleep()
            # next_joint_state = jointPositionsHome
            # controller.send_cmd(next_joint_state,time)
            # r.sleep()
      elif optAction == 1 : # pick(2)
         if oldAction != 1 :
            print("pick(2)")
            oldAction = 1
            controller.do_action(optAction)
            # next_joint_state = jointPositionsP2
            # controller.send_cmd(next_joint_state,time)
            # r.sleep()
            # next_joint_state = jointPositionsHome
            # controller.send_cmd(next_joint_state,time)
            # r.sleep()
      elif optAction == 2 : # pick(3)
         if oldAction != 2 :
            print("pick(3)")
            oldAction = 2
            controller.do_action(optAction)
            # next_joint_state = jointPositionsP3
            # controller.send_cmd(next_joint_state,time)
            # r.sleep()
            # next_joint_state = jointPositionsHome
            # controller.send_cmd(next_joint_state,time)
            # r.sleep()
      elif optAction == 3 : # place(1)
         if oldAction != 3 :
            print("place(1)")
            oldAction = 3
            controller.do_action(optAction)
            # next_joint_state = jointPositionsP1
            # controller.send_cmd(next_joint_state,time)
            # r.sleep()
            # next_joint_state = jointPositionsHome
            # controller.send_cmd(next_joint_state,time)
            # r.sleep()
      elif optAction == 4 : # place(2)
         if oldAction != 4 :
            print("place(2)")
            oldAction = 4
            controller.do_action(optAction)
            # next_joint_state = jointPositionsP2
            # controller.send_cmd(next_joint_state,time)
            # r.sleep()
            # next_joint_state = jointPositionsHome
            # controller.send_cmd(next_joint_state,time)
            # r.sleep()
      elif optAction == 5 : # place(3)
         if oldAction != 5 :
            print("place(3)")
            oldAction = 5
            controller.do_action(optAction)
            # next_joint_state = jointPositionsP3
            # controller.send_cmd(next_joint_state,time)
            # r.sleep()
            # next_joint_state = jointPositionsHome
            # controller.send_cmd(next_joint_state,time)
            # r.sleep()
         
   else:
      if oldAction != 10 :
         print("Goal reached")
         oldAction = 10

def planner(markovResults, initState) :

   policy = markovResults[0]
   goalID = markovResults[1]
   sequence = []
   nbActions = 6

   #search for initStateID
   for k in np.arange(len(markovMap)) :
      if initState in markovMap[k] :
         initStateID = markovMap[k][0]
         break

   #initStateID = searchForID(initState)
   currentState = initStateID
   for i in np.arange(nbActions) :
      if policy[currentState,i] == 1 :
         optAction = i 
   
   if markovMap[currentState][2] == optAction :
      postcondition = markovMap[currentState][3] 
   elif markovMap[currentState][4] == optAction :
      postcondition = markovMap[currentState][5]

   plannerDuo = [optAction, postcondition]
   return plannerDuo

if __name__ == '__main__' :
   rospy.init_node('planner')
   rate = rospy.Rate(1.0)
   # controller.times_sync.registerCallback(controller.state_subscriber)

   global markovResults
   markovResults = reinforcementLearning(demos)
   controller = PdCtrl()
   #inserer position de depart = home
   while not rospy.is_shutdown() :
      rospy.Subscriber('currentState', String, handler_state_msgs) #when a message is received, callback is invoked w/ message as 1st arg
      rate.sleep()