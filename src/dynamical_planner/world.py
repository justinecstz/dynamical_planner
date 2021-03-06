#!/usr/bin/env python
import tf
import numpy as np
import rospy
from std_msgs.msg import String

class World(object):
	def __init__(self):
		self.listener_markers = tf.TransformListener(True, rospy.Duration(5))
		self.listener_cubes = tf.TransformListener(True, rospy.Duration(5))
		self.ids_markers = [0, 1, 3] 
		self.human = 0
		self.visible_markers = []
		self.visible_cubes = []
		self.visible_items = []
		self.x_position_markers = []
		self.x_position_cubes = []
		self.y_position_markers = []
		self.y_position_cubes = []
		self.z_position_markers = []
		self.z_position_cubes = []
		self.x_position = []
		self.y_position = []
		self.z_position = []
		self.old_state = ""
		self.state = ""
		self.state_full = ""
		self.closed = 0
		self.visible = []

		self.prerecorded_x = []
		self.prerecorded_x_sorted = []


	def reinitialize(self):
		self.state = ""
		self.state_full = ""
		self.visible_markers = []
		self.visible_cubes = []
		self.x_position_markers = []
		self.x_position_cubes = []
		self.y_position_markers = []
		self.y_position_cubes = []
		self.z_position_markers = []
		self.z_position_cubes = []
		self.x_position = []
		self.y_position = []
		self.z_position = []
		self.visible = []


	def observe_world(self):
		for id_marker in self.ids_markers :
			full_id = "marker/" + str(id_marker)
			if self.listener_markers.canTransform("camera/1", full_id, rospy.Time.now() - rospy.Duration(1.0)) :
				self.visible_markers.append(id_marker)
				(position,quaternion) = self.listener_markers.lookupTransform("camera/1", full_id, rospy.Time(0))
				self.x_position_markers.append(position[0])
				self.y_position_markers.append(position[1])
				self.z_position_markers.append(position[2])

		if self.listener_cubes.canTransform("camera/1","green",rospy.Time.now() - rospy.Duration(1.0)) :
			self.visible_cubes.append("G")
			(position,quaternion) = self.listener_cubes.lookupTransform("camera/1", "green", rospy.Time(0))
			self.x_position_cubes.append(position[0]*0.0006-0.218) 
			self.y_position_cubes.append(position[1]*0.0007-0.1972)
			self.z_position_cubes.append(position[2])

		if self.listener_cubes.canTransform("camera/1", "blue", rospy.Time.now() - rospy.Duration(1.0)) :
			self.visible_cubes.append("B")
			(position,quaternion) = self.listener_cubes.lookupTransform("camera/1","blue", rospy.Time(0))
			self.x_position_cubes.append(position[0]*0.0006-0.218)
			self.y_position_cubes.append(position[1]*0.0007-0.1972)
			self.z_position_cubes.append(position[2])


		self.visible_items = self.visible_markers + self.visible_cubes
		self.x_position = self.x_position_markers + self.x_position_cubes
		self.y_position = self.y_position_markers + self.y_position_cubes
		self.z_position = self.z_position_markers + self.z_position_cubes


	def save_markers_pos(self):
		confirmation = raw_input("Please leave markers free. The program will process to the recording of the positions of the markers. Press y when ready. ")
		if confirmation == "y":
			for id_marker in self.ids_markers :
				full_id = "marker/" + str(id_marker)
				if self.listener_markers.canTransform("camera/1", full_id, rospy.Time.now() - rospy.Duration(1.0)) :
					self.visible_markers.append(id_marker)
					(position,quaternion) = self.listener_markers.lookupTransform("camera/1", full_id, rospy.Time(0))
					self.prerecorded_x.append(position[0])

			x = np.array(self.prerecorded_x) 
			x = np.argsort(x)

			length = [i for i in range(len(self.prerecorded_x))]
			for k in length:
				self.prerecorded_x_sorted.append(self.prerecorded_x[x[k]])

			print("Done.")


	def sort_visible_items(self):
		length = [i for i in range(len(self.visible_items))]
		for k in length :
			self.visible.append([self.visible_items[k],self.x_position[k],self.y_position[k],self.z_position[k]])

		x = np.array(self.x_position) 
		x = np.argsort(x)

		# y = np.array(world.y_position)
		# y = np.argsort(y)

		total_items = len(self.visible_markers) + len(self.visible_cubes)

		count = 0

		for k in np.arange(total_items) :
			while self.x_position[x[k]] != self.visible[count][1] :
			# while world.y_position[y[k]] != visible[count][2] :
				count = count + 1
			if self.visible[count][0] != "G" and self.visible[count][0] != "B" :
				self.state = self.state + "N"
			else :
				self.state = self.state + self.visible[count][0]
			count = 0 

	def full_artificial_state(self):
		if "B" in self.state and "G" in self.state:
			self.state_full = self.state + "N" 
		elif "B" in self.state and not "G" in self.state:
			if self.closed == 1 :
				self.state_full = self.state + "G"
			elif self.closed == 0 :
				self.state_full = self.state + "N"

		elif "G" in self.state and not "B" in self.state:
			if self.closed == 1 :
				self.state_full = self.state + "B"
			elif self.closed == 0 :
				self.state_full = self.state + "N"

		elif not "B" in self.state and not "G" in self.state:
			# if self.closed == 1 :
			# 	self.state_full = self.state + self.old_state[-2]
			# elif self.closed == 0 :
			# 	self.state_full = self.state + "N"   
			length_state = [i for i in range(len(self.old_state))] 
			if self.closed == 1:
				for i in length_state:
					if self.old_state[i] != "N":
						self.state_full = self.state + self.old_state[i]
			else:
				self.state_full = self.state + "N" 






