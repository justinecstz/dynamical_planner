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
# joint_positions_p1 = 1*np.array([-0.044405747733462106, 1.19814689840694254, 0.1520219301975148, -1.607911064798659, -0.07193516616802327, 0.3915826375219599, -1.568347410379195])
# joint_positions_p2 = 1*np.array([0.644405747733462106, 1.15814689840694254, 0.1520219301975148, -1.607911064798659, -0.07193516616802327, 0.3915826375219599, -1.568347410379195]) 
# joint_positions_p3 = 1*np.array([1.344405747733462106, 1.19814689840694254, 0.1520219301975148, -1.607911064798659, -0.07193516616802327, 0.3915826375219599, -1.568347410379195])
# joint_positions_home = 1*np.array([0.644405747733462106, 0.69814689840694254, 0.1520219301975148, -1.607911064798659, -0.07193516616802327, 0.7915826375219599, -1.568347410379195])
joint_position_handover = 1*np.array([0.644405747733462106, 1.30814689840694254, 0.1520219301975148, 0.407911064798659, -0.07193516616802327, 0.7915826375219599, -1.568347410379195])

# Real robot old positions
# joint_positions_p1 = 1*np.array([-0.4339979790845205, 1.3373235520341373, -0.1650918060481264, -0.39354840374588973, 0.16458329147693798, 1.4079477279662593, -1.570306351787338])
# joint_positions_p2 = 1*np.array([-0.1526515215095898, 1.186118107041075, -0.4308292301986005, -0.757119167631625, 0.37156025122477176, 1.2697712925938716, -1.5704178650184049])
# joint_positions_p3 = 1*np.array([0.008061009099187546, 1.140941414106668, -0.4140705499230182, -0.8524215590582609, 0.40469318154063466, 1.250843827247102, -1.5704030045215718])
# joint_positions_home = 1*np.array([-0.2561309292711379, 0.48341762688080325, -0.07932016919260708, -1.0536090715454243, -0.017271089745345972, 1.5119789493870104, -1.5704001283280475])

# Real robot new positions
joint_positions_p1 = 1*np.array([0.313771815355006, 0.7874442301146876, -0.35008257875556276, -1.605427980371526, 0.39355274195809514, 0.7924178032543101, -1.4842731591265332])
joint_positions_p2 = 1*np.array([0.5628616530365289, 0.8266034640114479, -0.343280991826965, -1.548290132212003, 0.36961525944794194, 0.787609672383395, -1.4840846472048561])
joint_positions_p3 = 1*np.array([0.7087980537375754, 0.9364973682354464, -0.3175523589760338, -1.3048212555591447, 0.32881175402628704, 0.9519498877320673, -1.483692343658399])
joint_positions_home = 1*np.array([0.5569787767738359, 0.2807509613788765, -0.2521163338340084, -1.4935594952802898, 0.06308649381713503, 1.3422390616692874, -1.528839315708223])

# current_target = joint_positions_home
# gripperCommand = 'o'
# q_old = joint_positions_home

# position_reached = 0
# actionPhase =  1
# gripperStatus = 0

class Planner(object):
	def __init__(self):
		self.current_action = "home"
		self.current_target = joint_positions_home

class Gripper(object):
	def __init__(self):
		self.grip_command = outputMsg.SModel_robot_output()
		self.pub_gripper = rospy.Publisher('SModelRobotOutput', outputMsg.SModel_robot_output, queue_size = 1);
		self.sub_gripper = rospy.Subscriber('SModelRobotInput', inputMsg.SModel_robot_input, self.get_gripper_msg)
		self.gripper_goal = "o"
		self.gripper_msg_POA = 0

	def get_gripper_msg(self,data):
		self.gripper_msg_POA = data.gPOA

class PdCtrl:
	def __init__(self):
		self.ctrl_freq = 200
		self.q_send = 1*np.array([-0.2561309292711379, 0.48341762688080325, -0.07932016919260708, -1.0536090715454243, -0.017271089745345972, 1.5119789493870104, -1.5704001283280475])
		self.q_old = 1*np.array([-0.2561309292711379, 0.48341762688080325, -0.07932016919260708, -1.0536090715454243, -0.017271089745345972, 1.5119789493870104, -1.5704001283280475])
		self.joint_names = ["iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"]
		self.joint_cmd_pub =  rospy.Publisher('/iiwa/PositionController/command', numpy_msg(Float64MultiArray),queue_size=10)
		self.joint_state_sub = rospy.Subscriber('iiwa/joint_states', numpy_msg(JointState), self.get_joint_state)
		self.joint_current_states = joint_positions_home 
		self.position_reached = 0

	def get_joint_state(self,data):
		self.joint_current_states = data.position

	def send_cmd(self,command): #command = Float64MultiArray
		next_joint_state = Float64MultiArray()

		next_joint_state.data = command 
		self.joint_cmd_pub.publish(next_joint_state)

	def lin_ds(self,current_joint_position, target_state):
		time = 0.02 #0.02
 		K = 6 #10

		delta_q = current_joint_position - self.q_old
		self.q_old = current_joint_position

		q_speed_max = [0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.001]
		error = current_joint_position-target_state  

		q_dot = -K*error

		for i in np.arange(7):
			if abs(q_dot[i]) > q_speed_max[i]:
				q_dot[i] = np.sign(q_dot[i])*q_speed_max[i]

		self.q_send = q_dot*time + self.q_old

       # graphs.listenerEndEff.waitForTransform('iiwa_link_ee','world', rospy.Time(), rospy.Duration(1))
       # (endEffPos,endEffQuat) = graphs.listenerEndEff.lookupTransform('world','iiwa_link_ee',rospy.Time(0))

       # timeSim = rospy.get_time()
       # graphs.save_data(self.q_send,error,q_dot,timeSim,endEffPos)

      # global q_star
      # global q_send
      # global q_old

      #  # k = 0.8
      #  # A = np.array([[k, 0, 0, 0, 0, 0, 0],
      #  #               [0, k, 0, 0, 0, 0, 0],
      #  #               [0, 0, k, 0, 0, 0, 0],
      #  #               [0, 0, 0, k, 0, 0, 0],
      #  #               [0, 0, 0, 0, k, 0, 0],
      #  #               [0, 0, 0, 0, 0, k, 0],
      #  #               [0, 0, 0, 0, 0, 0, k]])
      
      #  # B = current_joint_position - target_state
      #  # B = np.transpose(B)
      #  # #next_joint_state = np.matmul(A, B) + current_joint_position
      #  # next_joint_state = np.matmul(A, B) + target_state
      # alpha = 0.9
      # kp = 0.9
      # kd = 0
      # time = 0.100

      # delta_q = current_joint_position - q_old
      # q_old = current_joint_position
      # q_star = (1-alpha)*q_star + alpha*target_state
      # q_dot = -kp*(current_joint_position - q_star)
      # q_send = q_send + q_dot*time + kd*(delta_q/time)
      # print(q_send)
      # joint_values = [q_star,q_send]
      # return next_joint_state

	def do_action(self,target) :
        # global q_star
        # global q_send
		# global position_reached
    	# global current_action

		rtol = 1e-4
		atol = 1e-4
		tol = 5e-3

		# print(target)

        # time = rospy.Time.now()
		r = rospy.Rate(controller.ctrl_freq)

 		current_position = self.joint_current_states
 		# print(current_position)
		self.q_send = current_position
        # q_speed = current_position*0.005

		self.lin_ds(current_position, target)
		self.send_cmd(self.q_send)

        # current_position = joint_states_msg.position
        # q_star = current_position
        # q_send = current_position

        # controller.lin_ds(current_position, target)
        # controller.send_cmd(q_send,time)
        
        # joint_states_msg = rospy.wait_for_message('iiwa/joint_states', numpy_msg(JointState))
        # current_position = joint_states_msg.position
		diff_target = abs(target[0:6]-current_position[0:6])
		diff_home = abs(joint_positions_home[0:6]-current_position[0:6])
		# print(diff_home)
		# print(np.all(diff_home < tol))	
        # print(position_reached)
		# print(planner.current_action)
        # print(actionPhase)

		action_state = "not terminated"
		pub.publish(action_state)
		# print(np.amax(diff_target) < tol)

		if planner.current_action != "handover" :
			# if np.allclose(target[0:7],current_position[0:7], rtol, atol) and ((planner.current_target==joint_positions_p1).all() or \
	  #   				(planner.current_target==joint_positions_p2).all() or (planner.current_target==joint_positions_p3).all()) : #Contrainte ok
			if np.amax(diff_target) < tol and ((planner.current_target==joint_positions_p1).all() or \
	    				(planner.current_target==joint_positions_p2).all() or (planner.current_target==joint_positions_p3).all()) :

				# print("hello2")
				controller.position_reached = 1
				action_state = "terminated"
				pub.publish(action_state)
	            
				if gripper.gripper_goal == "c" :
		  #           # close gripper
		  			print("Closing gripper")
					gripper.grip_command.rACT = 1
					gripper.grip_command.rGTO = 1
					gripper.grip_command.rSPA = 255
					gripper.grip_command.rFRA = 150
					gripper.grip_command.rPRA = 255
					# gripper.pub_gripper.publish(gripper.grip_command)
					# while gripper.gripper_msg_POA < 200 :
					# 	print(gripper.gripper_msg_POA)
					# 	print("Closing gripper...")

						# gripper_msg = rospy.wait_for_message('SModelRobotInput', inputMsg.SModel_robot_input) #????

				elif gripper.gripper_goal == "o" :
		  #       	# open gripper
		  			print("Opening gripper")
					gripper.grip_command.rACT = 1
					gripper.grip_command.rGTO = 1
					gripper.grip_command.rSPA = 255
					gripper.grip_command.rFRA = 150
					gripper.grip_command.rPRA = 0
					# gripper.pub_gripper.publish(gripper.grip_command)
					# while gripper.gripper_msg_POA > 10:
					# 	print(gripper.gripper_msg_POA)
					# 	print("Opening gripper...")

	    	# elif controller.position_reached == 1 and np.allclose(joint_positions_home[0:7],current_position[0:7], rtol, atol)  : #pas ok
			# elif np.allclose(target[0:7],current_position[0:7], rtol, atol) and (planner.current_target==joint_positions_home).all() : # Contrainte ok
			elif np.amax(diff_home) < tol and (planner.current_target==joint_positions_home).all() :
					controller.position_reached = 0
					# print("hello")

		else:
			# Revoir condition
			if np.allclose(target[2:7],current_position[2:7], rtol, atol) and ((planner.current_target==joint_positions_p1).all() or \
						(planner.current_target==joint_positions_p2).all() or (planner.current_target==joint_positions_p3).all() or (planner.current_target==joint_positions_home).all()) :
				controller.position_reached = 1
				action_state = "terminated"
				pub.publish(action_state)	
		r.sleep()
            

def handler_action_msgs(data) :

	# global current_action
	# global current_target
	# global gripperCommand

	planner.current_action = data.data
	# print(planner.current_action)

	if planner.current_action == "pick1":
		if controller.position_reached == 0 :
			planner.current_target = joint_positions_p1
			gripper.gripper_goal = 'c'
		else:
			planner.current_target = joint_positions_home

	elif planner.current_action == "pick2":
		if controller.position_reached == 0 :
			planner.current_target = joint_positions_p2
			gripper.gripper_goal = 'c'
		else:
			planner.current_target = joint_positions_home

	elif planner.current_action == "pick3":
		if controller.position_reached == 0 :
			planner.current_target = joint_positions_p3
			gripper.gripper_goal = 'c'
		else:
			planner.current_target = joint_positions_home

	elif planner.current_action == "place1":
		if controller.position_reached == 0 :
			planner.current_target = joint_positions_p1
			gripper.gripper_goal = 'o'
		else:
			planner.current_target = joint_positions_home

	elif planner.current_action == "place2":
		if controller.position_reached == 0 :
			planner.current_target = joint_positions_p2
			gripper.gripper_goal = 'o'
		else:
			planner.current_target = joint_positions_home

	elif planner.current_action == "place3":
		if controller.position_reached == 0 :
			planner.current_target = joint_positions_p3
			gripper.gripper_goal = 'o'
		else:
			planner.current_target = joint_positions_home

	elif planner.current_action == "home" or planner.current_action == "wait":
		planner.current_target = joint_positions_home

	elif planner.current_action == "handover":
		planner.current_target = joint_position_handover

if __name__ == '__main__' :
	rospy.init_node('moveRobot')
	controller = PdCtrl()
	planner = Planner()
	gripper = Gripper()
	rate = rospy.Rate(200)
	pub = rospy.Publisher('action_state', String, queue_size = 10)

	gripper.grip_command.rACT = 1
	gripper.grip_command.rGTO = 1
	gripper.grip_command.rSPA = 255
	gripper.grip_command.rFRA = 150
	# rospy.sleep(2)
	# gripper.pub_gripper.publish(gripper.grip_command) #activate gripper
	# rospy.sleep(5)

	while not rospy.is_shutdown() :
		rospy.Subscriber('current_target', String, handler_action_msgs) #when a message is received, callback is invoked w/ message as 1st arg
		controller.do_action(planner.current_target)
		# print(controller.position_reached)
		rate.sleep()


