#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import String
# from robotiq_s_model_control.msg import _SModel_robot_input  as inputMsg
# from robotiq_s_model_control.msg import _SModel_robot_output  as outputMsg

class Gripper(object):
	def __init__(self):
		self.gripper_status = 0
		# self.sub_gripper = rospy.Subscriber("SModelRobotInput", inputMsg.SModel_robot_input, self.gripper_status_cb)
		self.sub_command = rospy.Subscriber("gripper_command", String, self.do_gripper_command)
		# self.grip_command = outputMsg.SModel_robot_output()
		# self.pub_gripper = rospy.Publisher("SModelRobotOutput", outputMsg.SModel_robot_output, queue_size = 1);
		self.gripper_goal = "o"
		print "initialized"

	def gripper_status_cb(self,data):
		self.gripper_status = data.gPOA
		print self.gripper_status
		# print "in callback"

	def do_gripper_command(self,data):
		
		command = data.data

		if command == "c":
			self.grip_command.rACT = 1
			self.grip_command.rGTO = 1
			self.grip_command.rSPA = 255
			self.grip_command.rFRA = 150
			self.grip_command.rPRA = 255
			self.pub_gripper.publish(self.grip_command)
			while self.gripper_status < 200 :
				print(self.gripper_status)
				print("Closing gripper...")
		elif command == "o":
			self.grip_command.rACT = 1
			self.grip_command.rGTO = 1
			self.grip_command.rSPA = 255
			self.grip_command.rFRA = 150
			self.grip_command.rPRA = 0
			self.pub_gripper.publish(self.grip_command)
			while self.gripper_status > 10:
				print(self.gripper_status)
				print("Opening gripper...")