#!/usr/bin/env python
import roslib
import rospy
import math
import tf
from std_msgs.msg import String
import numpy as np

markovMap = [[0, "GNBN", 0, 11, 2, 6], [1, "GBNN", 1, 6, 0, 10], [2, "BGNN", 0, 7, 1, 9], 
[3, "BNGN", 0, 8, 2, 9], [4, "NGBN", 1, 11, 2, 7], [5, "NBGN", 1, 8, 2, 10],
[6, "GNNB", 4, 1, 5, 0], [7, "NGNB", 3, 2, 5, 4], [8, "NNGB", 3, 3, 4, 5],
[9, "BNNG", 4, 2, 5, 3], [10, "NBNG", 3, 1, 5, 5], [11, "NNBG", 3, 0, 4, 4]]
#pour l'instant, la demo est pre-enregistree
demos = [["NGBN", "NNBG", "GNBN", "GNNB", "GBNN"], ["NBGN", "NBNG", "GBNN"], ["BGNN", "BNNG", "BNGN", "NNGB", "NBGN", "NBNG", "GBNN"]]
goal = demos[1][-1]

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
   policy = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
   oldPolicy = [i * 2 for i in policy]
   U = np.zeros((12, 2))
   U[:,0], U[:,1] = 1, 2
   gamma = 0.8 
   epsilon = 0.1
   sumsOnActions = np.zeros((1, 6))
   somme = 0

   #print(probaMatrices[5])
   #print(probaMatrices[5][1,1])

   for i in np.arange(len(markovMap)) :
   #for i in [0] :
      #print("hello")
      while oldPolicy[i] != policy[i] :
         while np.absolute(U[i, 0] - U[i, 1]) > epsilon :
            #print(np.absolute(U[i, 0] - U[i, 1]))
            U[i, 1] = U[i, 0]
            possibleActions = [markovMap[i][2], markovMap[i][4]]
            #print(possibleActions)
            for j in possibleActions :
               for k in np.arange(len(markovMap)) : #s'
                  somme = somme + probaMatrices[j][i, k] * (reward[i, j] + gamma * U[k, 1])
                  #print(somme)
            U[i, 0] = policy[i] * somme
            #print(U)
            somme = 0
         oldPolicy[i] = policy[i]
         for j in possibleActions :
            for k in np.arange(len(markovMap)) :
               somme = somme + probaMatrices[j][i, k] * (reward[i, j] + gamma * U[k, 1])
            sumsOnActions[0, j] = somme
            somme = 0
         policy[i] = np.argmax(sumsOnActions)
         sumsOnActions = np.zeros((1, 6))
   #print(policy)

   markovResults = [policy, goalID]

   return markovResults

def callback(data) :
	feedback = data
	
	precondition = feedback
	problem = 0

	if precondition != goal :
		if (compteur == 0 or compteur == 3) and problem == 0:
			plannerDuo = planner(markovResults, precondition)
			optAction = plannerDuo[0]
			postcondition = markovMap[plannerDuo[1]][1]

		if optAction == 0 :
			print("Action to realize is pick(1).")
		elif optAction == 1 :
			print("Action to realize is pick(2).")
		elif optAction == 2 :
			print("Action to realize is pick(3).")
		elif optAction == 3 :
			print("Action to realize is place(1).")
		elif optAction == 4 :
			print("Action to realize is place(2).")
		elif optAction == 5 :
			print("Action to realize is place(3).")

		#action is pick
		if optAction == 0 or optAction == 1 or optAction == 2 :
			#feedback = input("Give me the current feedback: ")
			#feedbackID = searchID(feedback)
			if feedback == precondition :
				print("Everything ok. DS continues action.")
				isInMap = 0
				for k in np.arange(len(markovMap)) :
					if precondition in markovMap[k] :
						currentState = markovMap[k][0]
						isInMap = 1
						break
				print(compteur)
				if isInMap == 0 :
					compteur = compteur + 1
				if compteur == 3 :
					print("Action succesfully done. Now entering Wait until acceptable state available.")
					precondition = waitForState()
					compteur = 0

			elif feedback != precondition and feedback != postcondition:
			#human intervention, problem
				problem = 1
				for i in np.arange(len(feedback)) :
					if feedback[i] != precondition[i] :
						positionProblem = i
						cubeProblem = feedback[i]
						print("{0} in Position {1} has become {2}".format(precondition[i], i+1, feedback[i]))
				# if positionProblem == optAction :
				# 	print("Cube to pick has disappeared! Action stopped.")
				# 	print("Now entering Wait until acceptable state available.")
				# 	precondition = waitForState()
				# else :
				if feedback[positionProblem] == "N" :
					print("Cube in position {0} has disappeared.".format(positionProblem+1))
					if positionProblem == optAction :
						print("A cube has disappeared where robot wants to pick its cube! Action stopped.")
						print("Now entering Wait until acceptable state available.")
						precondition = waitForState()
						problem = 0
					else :
						print("Pick can still happen. Action continues.")
						precondition = precondition[:positionProblem] + "N" + precondition[positionProblem+1:]
						# if compteur == 3 :
						# 	print("Action successfully done. Cube still missing. Now entering Wait until acceptable state available. ")
						# 	compteur = 0
						# 	precondition = waitForState()
				else :
					print("A cube has appeared in position {0}. Place can still happen. Action continues.".format(positionProblem+1))
					precondition = precondition[:positionProblem] + cubeProblem + precondition[positionProblem+1:]
				# print("Now entering Wait until acceptable state available.")
				# state = waitForState()
			elif feedback == postcondition :
				print("Postcondition reached. Action was successfully done. Planner to go on.")
				#precondition = postcondition

		#action is place
		else :
			#feedback = input("Give me the current feedback: ")
			#feedbackID = searchID(feedback)
			if feedback == precondition :
				print("Everything ok. DS continues action.")
				isInMap = 0
				for k in np.arange(len(markovMap)) :
					if precondition in markovMap[k] :
						currentState = markovMap[k][0]
						isInMap = 1
						break
	
				if isInMap == 0 :
					compteur = compteur + 1
				if compteur == 3 :
					print("Action succesfully done. Now entering Wait until acceptable state available.")
					precondition = waitForState()
					compteur = 0

			elif feedback != precondition and feedback != postcondition :
			#human intervention, problem
				problem = 1
				for i in np.arange(len(feedback)) :
					if feedback[i] != precondition[i] :
						positionProblem = i
						cubeProblem = feedback[i]
						print("{0} in Position {1} has become {2}".format(precondition[i], i+1, feedback[i]))
				# if positionProblem == optAction :
				# 	print("A cube has appeared where robot wants to place its cube! Action stopped.")
				# 	print("Now entering Wait until acceptable state available.")
				# 	precondition = waitForState()
				# else :
				if feedback[positionProblem] == "N" :
					print("A cube has disappeared in position {0}. Place can still happen. Action continues.".format(positionProblem+1))
					precondition = precondition[:positionProblem] + "N" + precondition[positionProblem+1:]
					# if compteur == 3 :
					# 	print("Action successfully done. Cube still missing. Now entering Wait until acceptable state available. ")
					# 	compteur = 0
					# 	precondition = waitForState()
				else :
					print("A cube has appeared in position {0}".format(positionProblem+1))
					if positionProblem == optAction - 3 :
						print("A cube has appeared where robot wants to place its cube! Action stopped.")
						print("Now entering Wait until acceptable state available.")
						precondition = waitForState()
						problem = 0
					else :
						print("Place can still happen. Action continues.")
						precondition = precondition[:positionProblem] + cubeProblem + precondition[positionProblem+1:]
			elif feedback == postcondition :
				print("Postcondition reached. Action was successfully done. Planner to go on.")
				#precondition = postcondition
	else :
		print("Goal reached.")

def planner(markovResults, initState) :

	policy = markovResults[0]
	goalID = markovResults[1]
	sequence = []

	#search for initStateID
	for k in np.arange(len(markovMap)) :
		if initState in markovMap[k] :
			initStateID = markovMap[k][0]
			break

	#initStateID = searchForID(initState)
	currentState = initStateID
	optAction = policy[currentState]
	if markovMap[currentState][2] == optAction :
		postcondition = markovMap[currentState][3] 
	elif markovMap[currentState][4] == optAction :
		postcondition = markovMap[currentState][5]

	plannerDuo = [optAction, postcondition]
	return plannerDuo

def waitForState() :
	isInMap = 0
	while isInMap == 0 :
		results = rospy.Subscriber('currentState', String, callback2)
		isInMap = results[0]
	return results[1]

def callback2(data)
	isInMap = 0
	#while isInMap == 0 :
		#feedback = input("Give me the state you see: ")
	for k in np.arange(len(markovMap)) :
		if feedback in markovMap[k] :
			currentState = markovMap[k][0]
			isInMap = 1
			state = markovMap[currentState][1]
			#print("State is now {0}. Show can go on.".format(state))
			break
	results = [isInMap, state]
	return results

if __name__ == '__main__' :
	rospy.init_node('watchDog')
	rate = rospy.Rate(1.0)

	while not rospy.is_shutdown() :
		#reception of current state published by getStateWorld
    	rospy.Subscriber('currentState', String, callback) #when a message is received, callback is invoked w/ message as 1st arg
    	rate.sleep()