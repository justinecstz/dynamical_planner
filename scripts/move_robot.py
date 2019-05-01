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


jointPositionsP1 = 1*np.array([-0.044405747733462106, 1.19814689840694254, 0.1520219301975148, -1.607911064798659, -0.07193516616802327, 0.3915826375219599, -1.568347410379195])
jointPositionsP2 = 1*np.array([0.644405747733462106, 1.15814689840694254, 0.1520219301975148, -1.607911064798659, -0.07193516616802327, 0.3915826375219599, -1.568347410379195]) 
jointPositionsP3 = 1*np.array([1.344405747733462106, 1.19814689840694254, 0.1520219301975148, -1.607911064798659, -0.07193516616802327, 0.3915826375219599, -1.568347410379195])
jointPositionsHome = 1*np.array([0.644405747733462106, 0.69814689840694254, 0.1520219301975148, -1.607911064798659, -0.07193516616802327, 0.7915826375219599, -1.568347410379195])

# pick1 = [jointPositionsP1, jointPositionsHome]
# pick2 = [jointPositionsP2, jointPositionsHome]
# pick3 = [jointPositionsP3, jointPositionsHome]
# place1 = [jointPositionsP1, jointPositionsHome]
# place2 = [jointPositionsP2, jointPositionsHome]
# place3 = [jointPositionsP3, jointPositionsHome]
# wait = [jointPositionsHome,jointPositionsHome]

currentTarget = jointPositionsHome

pick1 = jointPositionsP1
pick2 = jointPositionsP2
pick3 = jointPositionsP3
place1 = jointPositionsP1
place2 = jointPositionsP2
place3 = jointPositionsP3
wait = jointPositionsHome
home = jointPositionsHome

positionReached = 0
actionPhase =  1

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
      global q_star
      global q_send
       # k = 0.8
       # A = np.array([[k, 0, 0, 0, 0, 0, 0],
       #               [0, k, 0, 0, 0, 0, 0],
       #               [0, 0, k, 0, 0, 0, 0],
       #               [0, 0, 0, k, 0, 0, 0],
       #               [0, 0, 0, 0, k, 0, 0],
       #               [0, 0, 0, 0, 0, k, 0],
       #               [0, 0, 0, 0, 0, 0, k]])
      
       # B = current_joint_position - target_state
       # B = np.transpose(B)
       # #next_joint_state = np.matmul(A, B) + current_joint_position
       # next_joint_state = np.matmul(A, B) + target_state
      alpha = 0.9
      k = 0.9
      q_star = (1-alpha)*q_star + alpha*target_state
      q_dot = -k*(current_joint_position - q_star)
      q_send = q_send + q_dot*0.100
      # print(q_send)
      # joint_values = [q_star,q_send]
      # return next_joint_state

   def do_action(self,target) :
        global q_star
        global q_send
    	global positionReached
    	global currentAction
        global actionPhase

        rtol = 1e-3
        atol = 1e-3
        time = rospy.Time.now()
        r = rospy.Rate(controller.ctrl_freq)
        joint_states_msg = rospy.wait_for_message('iiwa/joint_states', numpy_msg(JointState))
        current_position = joint_states_msg.position
        q_star = current_position
        q_send = current_position

        controller.lin_ds(current_position, target)
        controller.send_cmd(q_send,time)
        
        # joint_states_msg = rospy.wait_for_message('iiwa/joint_states', numpy_msg(JointState))
        # current_position = joint_states_msg.position
      	
        # print(positionReached)
        print(currentAction)
        # print(actionPhase)

        actionState = "not terminated"
        pub.publish(actionState)

    	if np.allclose(target[2:7],current_position[2:7], rtol, atol) and ((currentTarget==pick1).all() or \
    				(currentTarget==pick2).all() or (currentTarget==pick3).all() or (currentTarget==place1).all() or (currentTarget==place2).all() or (currentTarget==place3).all()) :
    		
    		positionReached = 1
    		actionState = "terminated"
    		pub.publish(actionState)
    		# actionPhase == 2

    	elif positionReached == 1 and np.allclose(jointPositionsHome[2:7],current_position[2:7], rtol, atol)  :
    	# and (currentAction == "pick1" or \
    	# 			currentAction == "pick2" or currentAction == "pick3" or currentAction == "place1" or currentAction == "place2" or currentAction == "place3") :
    		positionReached = 0
    		# actionPhase = 1
    	
    	r.sleep()
            

def handler_action_msgs(data) :

	global currentAction
	global currentTarget
	currentAction = data.data
	# print(currentAction)

	if currentAction == "pick1":
		if positionReached == 0 :
			currentTarget = pick1
		else:
			currentTarget = home

	elif currentAction == "pick2":
		if positionReached == 0 :
			currentTarget = pick2
		else:
			currentTarget = home

	elif currentAction == "pick3":
		if positionReached == 0 :
			currentTarget = pick3
		else:
			currentTarget = home

	elif currentAction == "place1":
		if positionReached == 0 :
			currentTarget = place1
		else:
			currentTarget = home

	elif currentAction == "place2":
		if positionReached == 0 :
			currentTarget = place2
		else:
			currentTarget = home

	elif currentAction == "place3":
		if positionReached == 0 :
			currentTarget = place3
		else:
			currentTarget = home

	elif currentAction == "home":
		currentTarget = home

	elif currentAction == "wait":
		currentTarget = wait

if __name__ == '__main__' :
   rospy.init_node('moveRobot')
   controller = PdCtrl()
   rate = rospy.Rate(40)
   pub = rospy.Publisher('actionState', String, queue_size = 10)

   while not rospy.is_shutdown() :
      rospy.Subscriber('currentTarget', String, handler_action_msgs) #when a message is received, callback is invoked w/ message as 1st arg
      controller.do_action(currentTarget)
      rate.sleep()


