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
# from robotiq_s_model_control.msg import _SModel_robot_input  as inputMsg
# from robotiq_s_model_control.msg import _SModel_robot_output  as outputMsg
from dynamical_planner.gripper import Gripper
from dynamical_planner.pdctrl import PdCtrl
from dynamical_planner.action import Action

            
def handler_action_msgs(data) :

	controller.current_action = data.data
	# if controller.position_reached == 1 and action.priority == 0 :
	# 	controller.current_action = action.old_action
	# elif controller.position_reached == 0 :

	# rospy.loginfo(data)
	
	if controller.current_action == "pick1":
		if controller.position_reached == 0 :
			controller.current_target = controller.joint_positions_p1
			action.old_action = "pick1"
			# gripper.gripper_goal = 'c'
		else:
			controller.current_target = controller.joint_positions_home

	elif controller.current_action == "pick2":
		if controller.position_reached == 0 :
			controller.current_target = controller.joint_positions_p2
			action.old_action = "pick2"
			# gripper.gripper_goal = 'c'
		else:
			controller.current_target = controller.joint_positions_home

	elif controller.current_action == "pick3":
		if controller.position_reached == 0 :
			controller.current_target = controller.joint_positions_p3
			action.old_action = "pick3"
			# gripper.gripper_goal = 'c'
		else:
			controller.current_target = controller.joint_positions_home

	elif controller.current_action == "place1":
		if controller.position_reached == 0 :
			controller.current_target = controller.joint_positions_p1
			action.old_action = "place1"
			# gripper.gripper_goal = 'o'
		else:
			controller.current_target = controller.joint_positions_home

	elif controller.current_action == "place2":
		if controller.position_reached == 0 :
			controller.current_target = controller.joint_positions_p2
			action.old_action = "place2"
			# gripper.gripper_goal = 'o'
		else:
			controller.current_target = controller.joint_positions_home

	elif controller.current_action == "place3":
		if controller.position_reached == 0 :
			controller.current_target = controller.joint_positions_p3
			action.old_action = "place3"
			# gripper.gripper_goal = 'o'
		else:
			controller.current_target = controller.joint_positions_home

	# elif controller.current_action == "home" or controller.current_action == "wait":
	# 	controller.current_target = controller.joint_positions_home
	# 	action.old_action = "wait"
	elif controller.current_action == "home":
		controller.current_target = controller.joint_positions_home
		action.old_action = "home"

	elif controller.current_action == "wait":
		controller.current_target = controller.joint_positions_home
		action.old_action = "wait"


	elif controller.current_action == "handover":
		controller.current_target = controller.joint_positions_handover

if __name__ == '__main__' :
	rospy.init_node('moveRobot')
	controller = PdCtrl()
	gripper = Gripper()
	action = Action()
	rate = rospy.Rate(200)

	rospy.Subscriber('current_target', String, handler_action_msgs)

	# gripper.grip_command.rACT = 1
	# gripper.grip_command.rGTO = 1
	# gripper.grip_command.rSPA = 255
	# gripper.grip_command.rFRA = 150
	# rospy.sleep(2)
	# gripper.pub_gripper.publish(gripper.grip_command) #activate gripper
	# rospy.sleep(5)

	while not rospy.is_shutdown() :
		controller.do_action(controller.current_target)
		rate.sleep()


