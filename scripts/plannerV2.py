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
problem = 0
terminated = 0
pbOnPos = 0
currentTarget = "wait"
handover = 0
reachedHandover = 0

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
   global precondition
   global problem
   global currentTarget
   global oldCurrentTarget
   global terminated
   global pbOnPos 
   global reachedHandover

   isInMarkovMap = 0
   state = data.data

   if handover == 0:
      reachedHandover = 0
      #search if state is in Markov Map
      for k in np.arange(len(markovMap)) :
         if state in markovMap[k] :
            isInMarkovMap = 1
            precondition = state
            break

      if isInMarkovMap == 1:
         problem = 0  
         pbOnPos = 0    

         if state != goal :
            plannerDuo = planner(markovResults,state)
            optAction = plannerDuo[0]
            
            if optAction == 0 : 
               print("pick(1)")
               currentTarget = "pick1"
               if oldAction != 0 :
                  oldAction = 0

            elif optAction == 1 : 
               print("pick(2)")
               currentTarget = "pick2"
               if oldAction != 1 :
                  oldAction = 1
       
            elif optAction == 2 :
               print("pick(3)")
               currentTarget = "pick3"
               if oldAction != 2 :
                  oldAction = 2
         
            elif optAction == 3 : 
               print("place(1)")
               currentTarget = "place1"
               if oldAction != 3 :
                  oldAction = 3
          
            elif optAction == 4 : 
               print("place(2)")
               currentTarget = "place2"
               if oldAction != 4 :
                  oldAction = 4
           
            elif optAction == 5 : 
               print("place(3)")
               currentTarget = "place3"
               if oldAction != 5 :
                  oldAction = 5
               
         else:
            if oldAction != 10 :
               currentTarget = "home"
               print("Goal reached")
               oldAction = 10

      #if state is not in Markov Map, a problem happened
      elif isInMarkovMap == 0 :
         if problem == 0 :
            problem = 1

            #check what happened
            for i in np.arange(len(state)) :
               if state[i] != precondition[i] :
                  wrongLetterPos = i

            if precondition[wrongLetterPos] == "N" :
               print("A cube appeared in position {0}".format(wrongLetterPos+1))
            else :
               print("A cube was stolen in position {0}".format(wrongLetterPos+1))
            # print(oldAction)
            # print(currentTarget)
            #action was pick
            if oldAction == 0 or oldAction == 1 or oldAction == 2 :
               #cube that robot wanted to pick has been stolen
               if oldAction == wrongLetterPos :
                  print("The cube that shoud be picked has disappeared!")
                  print("Waiting...")
                  # controller.do_action(6) #action 6 = wait
                  currentTarget = "wait"
                  pbOnPos = 1
               # else :
               # #    print("hello")
               #    controller.do_action(optAction)

            #action was place
            elif oldAction == 3 or oldAction == 4 or oldAction == 5 :
               if oldAction == wrongLetterPos + 3 :
                  print("A cube appeared where the robot wants to place the cube!")
                  print("Waiting...")
                  # controller.do_action(6)
                  # currentTarget = "wait"
                  pbOnPos = 1
               # else :
               #    controller.do_action(optAction)
         elif problem == 1 and terminated == 0 :   
            # and pbOnPos == 0:
            print("Action goes on...")

         elif problem == 1 and terminated == 1 :
            print("Waiting...")
            # controller.do_action(6) 
            currentTarget = "wait"

   else :
      if reachedHandover == 0 :
         if state[-1] == "N" :
            if state[0] == "B" :
               currentTarget = "pick1"
               print("pick(1)")
            elif state[1] == "B":
               currentTarget = "pick2"
               print("pick(2)")
            elif state[2] == "B":
               currentTarget = "pick3"
               print("pick(3)")
         elif state[-1] == "B":
            currentTarget = "home"
            print("home")

      if terminated == 1 :
         currentTarget = "handover"
         print("handover")
         reachedHandover = 1


   pub.publish(currentTarget)

def action_state(data):
   
   global terminated
   action_state = data.data

   if action_state == "terminated":
      terminated = 1
   else :
      terminated = 0

def handover_func(data):
   
   global handover
   handover_state = data.data

   if handover_state == "handover":
      handover = 1
   else :
      handover = 0

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
   currentTarget = "home"
   pub = rospy.Publisher('currentTarget', String, queue_size = 10)
   # controller = PdCtrl()
   #inserer position de depart = home
   while not rospy.is_shutdown() :
      rospy.Subscriber('handover', String, handover_func)
      rospy.Subscriber('currentState', String, handler_state_msgs) #when a message is received, callback is invoked w/ message as 1st arg
      rospy.Subscriber('actionState', String, action_state)
      rate.sleep()