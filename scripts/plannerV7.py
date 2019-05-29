#!/usr/bin/env python
import roslib
import rospy
import math
import tf
from std_msgs.msg import String
import numpy as np
from dynamical_planner.markov import Markov
from dynamical_planner.action import Action

def planner(markovResults,init_state) :

   # print(markov.markov_results)
   policy = markov.markov_results[0]
   goal_id = markov.markov_results[1]
   sequence = []
   nb_actions = 6

   #search for init_state_id
   length = [i for i in range(len(markov.markov_map))]
   for k in length:
     if init_state in markov.markov_map[k] :
        init_state_id = markov.markov_map[k][0]
        break

   #init_state_id = searchForID(init_state)
   current_state = init_state_id
   for i in np.arange(nb_actions) :
     if policy[current_state,i] == 1 :
        opt_action = i 

   if markov.markov_map[current_state][2] == opt_action :
     postcondition = markov.markov_map[current_state][3] 
   elif markov.markov_map[current_state][4] == opt_action :
     postcondition = markov.markov_map[current_state][5]

   planner_duo = [opt_action, postcondition]
   return planner_duo


def handler_state_msgs(data) :

   is_in_markov_map = 0
   state = data.data

   if action.handover == 0:
     action.reached_handover = 0

     #search if state is in Markov Map
     for k in np.arange(len(markov.markov_map)) :
        if state in markov.markov_map[k] :
           is_in_markov_map = 1
           break

     if is_in_markov_map == 1:
     
        if state != markov.goal :

           planner_duo = planner(markov.markov_results,state)
           opt_action = planner_duo[0]
         
           if opt_action == 0 : 
              print("pick(1)")
              action.current_target = "pick1"

           elif opt_action == 1 : 
              print("pick(2)")
              action.current_target = "pick2"
      
           elif opt_action == 2 :
              print("pick(3)")
              action.current_target = "pick3"
        
           elif opt_action == 3 : 
              print("place(1)")
              action.current_target = "place1"
         
           elif opt_action == 4 : 
              print("place(2)")
              action.current_target = "place2"
          
           elif opt_action == 5 : 
              print("place(3)")
              action.current_target = "place3"

           if state != action.precondition : 
              if state == action.postcondition : #reached current goal
                if action.terminated == 0 : #robot has not reached home yet
                   pub_target.publish(action.old_target)
                elif action.terminated == 1 : #robot has reached home
                   pub_target.publish(action.current_target)
                   action.old_target = action.current_target
                   action.old_action = opt_action
                   action.precondition = state
                   action.postcondition = markov.markov_map[planner_duo[1]][1]

              elif state != action.postcondition : #jump in MDP
                   pub_target.publish(action.current_target)
                   action.precondition = state
                   action.postcondition = markov.markov_map[planner_duo[1]][1]

                   for i in np.arange(len(state)-1) :
                      if state[i] != action.precondition_pb[i] :
                         wrong_letter_pos = i

                   if wrong_letter_pos == 0 or wrong_letter_pos == 1 or wrong_letter_pos == 2 :

                      if action.precondition_pb[wrong_letter_pos] == "N" :
                         print("A cube appeared in position {0}".format(wrong_letter_pos+1))

                      if action.old_action == 3 or action.old_action == 4 or action.old_action == 5 :
                         if action.old_action == wrong_letter_pos + 3 :
                            print("A cube appeared where the robot wants to place the cube!")
                   
                   action.old_action = opt_action
                   action.old_target = action.current_target

           
           elif state == action.precondition:
              pub_target.publish(action.current_target)


           # if action.problem == 1 : #just recovering from a problem
           #    pub_target.publish(action.current_target)
           #    action.problem = 0 

        else:
           # if action.old_action != 10 :
            action.current_target = "home"
            print("Goal reached")
            action.old_action = 10
            pub_target.publish(action.current_target)

     #if state is not in Markov Map, a problem happened
     elif is_in_markov_map == 0 :
        # if action.problem == 0 :
        #    action.problem = 1

           #check what happened
       length = np.arange(len(state))
       for i in length :
          if state[i] != action.precondition[i] :
            wrong_letter_pos = i

       if action.precondition[wrong_letter_pos] != "N" :
          print("A cube was stolen in position {0}".format(wrong_letter_pos+1))

       if action.old_action == 0 or action.old_action == 1 or action.old_action == 2 :
          #cube that robot wanted to pick has been stolen
          if action.old_action == wrong_letter_pos :
             print("The cube that shoud be picked has disappeared!")
             # print("Waiting...")
             action.current_target = "wait"
             action.pb_on_pos = 1

        # elif action.problem == 1 and action.terminated == 0 : 
       if action.terminated == 0 :  
          print("Action goes on...")

        # elif action.problem == 1 and action.terminated == 1 :
       elif action.terminated == 1 :
          print("Waiting...")
          action.current_target = "wait"

       pub_target.publish(action.current_target)
       action.precondition_pb = state

   else :

     if state[-1] == "N" :
        if state[0] == "B" :
           action.current_target = "pick1"
           print("pick(1)")
        elif state[1] == "B":
           action.current_target = "pick2"
           print("pick(2)")
        elif state[2] == "B":
           action.current_target = "pick3"
           print("pick(3)")
      
     elif state[-1] == "B":
        action.current_target = "handover"
        print("handover")

     if action.terminated == 1 and state[3] == "B":
        action.current_target = "handover"

     pub_target.publish(action.current_target)


if __name__ == '__main__' :
   rospy.init_node('planner')
   markov = Markov()
   action = Action()
   rospy.Subscriber('current_state', String, handler_state_msgs)
   pub_target = rospy.Publisher('current_target', String, queue_size = 10)
   pub_target.publish(action.current_target)

   rate = rospy.Rate(1.0)
   markov.reinforcement_learning()
   while not rospy.is_shutdown() :
      rate.sleep()