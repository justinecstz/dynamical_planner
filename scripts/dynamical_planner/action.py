#!/usr/bin/env python
import roslib
import rospy
import math
import tf
from std_msgs.msg import String
import numpy as np

class Action(object):
	def __init__(self):
		self.old_action = 10
		self.precondition = ""
		self.problem = 0
		self.terminated = 0
		self.pb_on_pos = 0
		self.current_target = "home"
		self.handover = 0
		self.reached_handover = 0
		rospy.Subscriber('action_state', String, self.action_state)
		rospy.Subscriber('handover', String, self.handover_func)


	def action_state(self,data):

		action_state = data.data

		if action_state == "terminated":
		  self.terminated = 1
		else :
		  self.terminated = 0

	def handover_func(self,data):

		handover_state = data.data

		if handover_state == "handover":
		  self.handover = 1
		else :
		  self.handover = 0



