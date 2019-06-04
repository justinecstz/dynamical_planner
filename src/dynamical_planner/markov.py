#!/usr/bin/env python
import numpy as np

class Markov(object):
	def __init__(self):
		self.markov_map = [[0, "GNBN", 0, 11, 2, 6], [1, "GBNN", 1, 6, 0, 10], [2, "BGNN", 0, 7, 1, 9], 
		[3, "BNGN", 0, 8, 2, 9], [4, "NGBN", 1, 11, 2, 7], [5, "NBGN", 1, 8, 2, 10],
		[6, "GNNB", 4, 1, 5, 0], [7, "NGNB", 3, 2, 5, 4], [8, "NNGB", 3, 3, 4, 5],
		[9, "BNNG", 4, 2, 5, 3], [10, "NBNG", 3, 1, 5, 5], [11, "NNBG", 3, 0, 4, 4]]

		self.demos = [["GBNN","GNNB","GNBN", "NNBG", "NGBN", "NGNB", "BGNN"],["GBNN","NBNG","NBGN","NNGB", "BNGN","BNNG","BGNN"]]
		# self.demos = [["BGNN","NGNB","NGBN","NNBG","GNBN","GNNB","GBNN"],["BNNG","BNGN","NNGB","NBGN","NBNG","GBNN"]]
		self.goal = self.demos[0][-1]
		self.markov_results = []

	def reinforcement_learning(self) :

		#definition of probability matrices for the 12 states and 6 actions
		proba_matrices = [np.zeros((14, 14)), np.zeros((14, 14)), np.zeros((14, 14)), np.zeros((14, 14)), np.zeros((14, 14)), np.zeros((14, 14))]

		#action 0: pick(1)
		proba_matrices[0][(0, 11)], proba_matrices[0][(1, 10)], proba_matrices[0][(2, 7)], proba_matrices[0][(3, 8)] = 1, 1, 1, 1
		proba_matrices[0][(4, 4)], proba_matrices[0][(5, 5)], proba_matrices[0][(6, 6)], proba_matrices[0][(7, 7)],  = 1, 1, 1, 1
		proba_matrices[0][(8, 8)], proba_matrices[0][(9, 9)], proba_matrices[0][(10, 10)], proba_matrices[0][(11, 11)],  = 1, 1, 1, 1

		#action 1: pick(2)
		proba_matrices[1][(0, 0)], proba_matrices[1][(1, 6)], proba_matrices[1][(2, 9)], proba_matrices[1][(3, 3)] = 1, 1, 1, 1
		proba_matrices[1][(4, 11)], proba_matrices[1][(5, 8)], proba_matrices[1][(6, 6)], proba_matrices[1][(7, 7)],  = 1, 1, 1, 1
		proba_matrices[1][(8, 8)], proba_matrices[1][(9, 9)], proba_matrices[1][(10, 10)], proba_matrices[1][(11, 11)],  = 1, 1, 1, 1

		#action 2: pick(3)
		proba_matrices[2][(0, 6)], proba_matrices[2][(1, 1)], proba_matrices[2][(2, 2)], proba_matrices[2][(3, 9)] = 1, 1, 1, 1
		proba_matrices[2][(4, 7)], proba_matrices[2][(5, 10)], proba_matrices[2][(6, 6)], proba_matrices[2][(7, 7)],  = 1, 1, 1, 1
		proba_matrices[2][(8, 8)], proba_matrices[2][(9, 9)], proba_matrices[2][(10, 10)], proba_matrices[2][(11, 11)],  = 1, 1, 1, 1

		#action 3: place(1)
		proba_matrices[3][(0, 0)], proba_matrices[3][(1, 1)], proba_matrices[3][(2, 2)], proba_matrices[3][(3, 3)] = 1, 1, 1, 1
		proba_matrices[3][(4, 4)], proba_matrices[3][(5, 5)], proba_matrices[3][(6, 6)], proba_matrices[3][(7, 2)],  = 1, 1, 1, 1
		proba_matrices[3][(8, 3)], proba_matrices[3][(9, 9)], proba_matrices[3][(10, 1)], proba_matrices[3][(11, 0)],  = 1, 1, 1, 1

		#action 4: place(2)
		proba_matrices[4][(0, 0)], proba_matrices[4][(1, 1)], proba_matrices[4][(2, 2)], proba_matrices[4][(3, 3)] = 1, 1, 1, 1
		proba_matrices[4][(4, 4)], proba_matrices[4][(5, 5)], proba_matrices[4][(6, 1)], proba_matrices[4][(7, 7)],  = 1, 1, 1, 1
		proba_matrices[4][(8, 5)], proba_matrices[4][(9, 2)], proba_matrices[4][(10, 10)], proba_matrices[4][(11, 4)],  = 1, 1, 1, 1

		#action 5: place(3)
		proba_matrices[5][(0, 0)], proba_matrices[5][(1, 1)], proba_matrices[5][(2, 2)], proba_matrices[5][(3, 3)] = 1, 1, 1, 1
		proba_matrices[5][(4, 4)], proba_matrices[5][(5, 5)], proba_matrices[5][(6, 0)], proba_matrices[5][(7, 4)],  = 1, 1, 1, 1
		proba_matrices[5][(8, 8)], proba_matrices[5][(9, 3)], proba_matrices[5][(10, 5)], proba_matrices[5][(11, 11)],  = 1, 1, 1, 1

		#definition of the reward matrix
		reward = np.zeros((12, 6))

		length_markov = [i for i in range(len(self.markov_map))]

		for k in length_markov:
		  if self.goal in self.markov_map[k] :
		     goal_id = self.markov_map[k][0]
		     break

		#conversion states' names into IDs
		index_demo = np.arange(len(self.demos))
		demos_id = []

		for i in index_demo :
		  index_current_demo = np.arange(len(self.demos[i]))
		  current_demo = []
		  for j in index_current_demo :
		     state = self.demos[i][j]

		     for k in np.arange(len(self.markov_map)) :
		        if state in self.markov_map[k] :
		           current_demo.append(self.markov_map[k][0])
		           break

		  demos_id.append(current_demo)

		#construction reward matrix
		for i in index_demo :
		  index_current_demo = np.arange(len(self.demos[i]))
		  for j in index_current_demo :
		     state_id = demos_id[i][j]
		     if state_id != goal_id :
		        if demos_id[i][j + 1] == self.markov_map[state_id][3] :
		           reward[state_id, self.markov_map[state_id][2]] = reward[state_id, self.markov_map[state_id][2]] + 2
		        elif demos_id[i][j + 1] == self.markov_map[state_id][5] :
		           reward[state_id, self.markov_map[state_id][4]] = reward[state_id, self.markov_map[state_id][4]] + 2

		#construction of policy
		policy = np.zeros((12,6))
		old_policy = np.ones((12,6))

		nb_actions = 6
		all_actions = [i for i in range(nb_actions)]
		U = np.zeros((12, 2))
		U[:,0], U[:,1] = 1, 2
		gamma = 0
		epsilon = 0.1
		sums_on_actions = np.zeros((1, 6))
		sum_1 = 0
		sum_2 = 0

		while not np.allclose(old_policy, policy) :
		  for i in length_markov :
		     while np.absolute(U[i, 0] - U[i, 1]) > epsilon :
		        U[i, 1] = U[i, 0]
		        for j in all_actions :
		            for k in length_markov : #s'
		                sum_1 = sum_1 + proba_matrices[j][i, k] * (reward[i, j] + gamma * U[k, 1])
		            sum_2 = sum_2 + sum_1 * policy[i,j]
		        U[i, 0] = sum_2
		        sum_1 = 0
		        sum_2 = 0
		     old_policy[i,:] = policy[i,:]
		     for j in all_actions :
		        for k in length_markov :
		           sum_1 = sum_1 + proba_matrices[j][i, k] * (reward[i, j] + gamma * U[k, 1])
		        sums_on_actions[0, j] = sum_1
		        sum_1 = 0
		     policy[i,:] = 0
		     policy[i,np.argmax(sums_on_actions)] = 1 
		     sums_on_actions = np.zeros((1, 6))

		self.markov_results = [policy, goal_id]