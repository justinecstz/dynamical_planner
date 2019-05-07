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
from robotiq_s_model_control.msg import _SModel_robot_input  as inputMsg
from robotiq_s_model_control.msg import _SModel_robot_output  as outputMsg

# Simulation
# jointPositionsP1 = 1*np.array([-0.044405747733462106, 1.19814689840694254, 0.1520219301975148, -1.607911064798659, -0.07193516616802327, 0.3915826375219599, -1.568347410379195])
# jointPositionsP2 = 1*np.array([0.644405747733462106, 1.15814689840694254, 0.1520219301975148, -1.607911064798659, -0.07193516616802327, 0.3915826375219599, -1.568347410379195]) 
# jointPositionsP3 = 1*np.array([1.344405747733462106, 1.19814689840694254, 0.1520219301975148, -1.607911064798659, -0.07193516616802327, 0.3915826375219599, -1.568347410379195])
# jointPositionsHome = 1*np.array([0.644405747733462106, 0.69814689840694254, 0.1520219301975148, -1.607911064798659, -0.07193516616802327, 0.7915826375219599, -1.568347410379195])
jointPositionHandover = 1*np.array([0.644405747733462106, 1.30814689840694254, 0.1520219301975148, 0.407911064798659, -0.07193516616802327, 0.7915826375219599, -1.568347410379195])

# Real robot
jointPositionsP1 = 1*np.array([-0.4339979790845205, 1.3373235520341373, -0.1650918060481264, -0.39354840374588973, 0.16458329147693798, 1.4079477279662593, -1.570306351787338])
jointPositionsP2 = 1*np.array([-0.1526515215095898, 1.186118107041075, -0.4308292301986005, -0.757119167631625, 0.37156025122477176, 1.2697712925938716, -1.5704178650184049])
jointPositionsP3 = 1*np.array([0.008061009099187546, 1.140941414106668, -0.4140705499230182, -0.8524215590582609, 0.40469318154063466, 1.250843827247102, -1.5704030045215718])
jointPositionsHome = 1*np.array([-0.2561309292711379, 0.48341762688080325, -0.07932016919260708, -1.0536090715454243, -0.017271089745345972, 1.5119789493870104, -1.5704001283280475])


# pick1 = [jointPositionsP1, jointPositionsHome]
# pick2 = [jointPositionsP2, jointPositionsHome]
# pick3 = [jointPositionsP3, jointPositionsHome]
# place1 = [jointPositionsP1, jointPositionsHome]
# place2 = [jointPositionsP2, jointPositionsHome]
# place3 = [jointPositionsP3, jointPositionsHome]
# wait = [jointPositionsHome,jointPositionsHome]

currentTarget = jointPositionsHome
gripperCommand = 'o'
q_old = jointPositionsHome

# pick1 = jointPositionsP1
# pick2 = jointPositionsP2
# pick3 = jointPositionsP3
# place1 = jointPositionsP1
# place2 = jointPositionsP2
# place3 = jointPositionsP3
# wait = jointPositionsHome
# home = jointPositionsHome

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
      global q_old

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
      kp = 0.9
      kd = 0
      time = 0.100

      delta_q = current_joint_position - q_old
      q_old = current_joint_position
      q_star = (1-alpha)*q_star + alpha*target_state
      q_dot = -kp*(current_joint_position - q_star)
      q_send = q_send + q_dot*time + kd*(delta_q/time)
      # print(q_send)
      # joint_values = [q_star,q_send]
      # return next_joint_state

   def do_action(self,target) :
        global q_star
        global q_send
    	global positionReached
    	global currentAction
        global actionPhase

        rtol = 1e-2
        atol = 1e-2
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

        if currentAction != "handover" :
	    	if np.allclose(target[2:7],current_position[2:7], rtol, atol) and ((currentTarget==jointPositionsP1).all() or \
	    				(currentTarget==jointPositionsP2).all() or (currentTarget==jointPositionsP3).all()) :
	    		
	    		positionReached = 1
	    		actionState = "terminated"
	    		pub.publish(actionState)
	    		pubGripper.publish(gripperCommand)
	    		gripper_msg = rospy.wait_for_message('SModelRobotInput', inputMsg.SModel_robot_input)
	    		if gripperCommand == 'c' :
	    			while gripper_msg.gPOA < 225 :
	    				gripper_msg = rospy.wait_for_message('SModelRobotInput', inputMsg.SModel_robot_input)
	    		elif gripperCommand == 'o' :
	    			while gripper_msg.gPOA > 10:
	    				gripper_msg = rospy.wait_for_message('SModelRobotInput', inputMsg.SModel_robot_input)

	    	elif positionReached == 1 and np.allclose(jointPositionsHome[2:7],current_position[2:7], rtol, atol)  :
	    	# and (currentAction == "pick1" or \
	    	# 			currentAction == "pick2" or currentAction == "pick3" or currentAction == "place1" or currentAction == "place2" or currentAction == "place3") :
	    		positionReached = 0
	    		# actionPhase = 1

		else:
			if np.allclose(target[2:7],current_position[2:7], rtol, atol) and ((currentTarget==jointPositionsP1).all() or \
						(currentTarget==jointPositionsP2).all() or (currentTarget==jointPositionsP3).all() or (currentTarget==jointPositionsHome).all()) :
				positionReached = 1
				actionState = "terminated"
				pub.publish(actionState)	
		r.sleep()
            

def handler_action_msgs(data) :

	global currentAction
	global currentTarget
	global gripperCommand

	currentAction = data.data
	# print(currentAction)

	if currentAction == "pick1":
		if positionReached == 0 :
			currentTarget = jointPositionsP1
			gripperCommand = 'c'
		else:
			currentTarget = jointPositionsHome

	elif currentAction == "pick2":
		if positionReached == 0 :
			currentTarget = jointPositionsP2
			gripperCommand = 'c'
		else:
			currentTarget = jointPositionsHome

	elif currentAction == "pick3":
		if positionReached == 0 :
			currentTarget = jointPositionsP3
			gripperCommand = 'c'
		else:
			currentTarget = jointPositionsHome

	elif currentAction == "place1":
		if positionReached == 0 :
			currentTarget = jointPositionsP1
			gripperCommand = 'o'
		else:
			currentTarget = jointPositionsHome

	elif currentAction == "place2":
		if positionReached == 0 :
			currentTarget = jointPositionsP2
			gripperCommand = 'o'
		else:
			currentTarget = jointPositionsHome

	elif currentAction == "place3":
		if positionReached == 0 :
			currentTarget = jointPositionsP3
			gripperCommand = 'o'
		else:
			currentTarget = jointPositionsHome

	elif currentAction == "home" or currentAction == "wait":
		currentTarget = jointPositionsHome

	elif currentAction == "handover":
		currentTarget = jointPositionHandover

if __name__ == '__main__' :
   rospy.init_node('moveRobot')
   controller = PdCtrl()
   rate = rospy.Rate(40)
   pub = rospy.Publisher('actionState', String, queue_size = 10)
   pubGripper = rospy.Publisher('SModelRobotOutput', outputMsg.SModel_robot_output);
   pubGripper.publish('a') #activate gripper

   while not rospy.is_shutdown() :
      rospy.Subscriber('currentTarget', String, handler_action_msgs) #when a message is received, callback is invoked w/ message as 1st arg
      controller.do_action(currentTarget,gripperCommand)
      rate.sleep()


