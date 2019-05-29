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


	def observe_world(self):
		for id_marker in self.ids_markers :
			full_id = "marker/" + str(id_marker)
			if self.listener_markers.canTransform("camera/0", full_id, rospy.Time.now() - rospy.Duration(1.0)) :
				self.visible_markers.append(id_marker)
				(position,quaternion) = self.listener_markers.lookupTransform("camera/0", full_id, rospy.Time(0))
				self.x_position_markers.append(position[0])
				self.y_position_markers.append(position[1])
				self.z_position_markers.append(position[2])

		if self.listener_cubes.canTransform("camera/0","green",rospy.Time.now() - rospy.Duration(1.0)) :
			self.visible_cubes.append("G")
			(position,quaternion) = self.listener_cubes.lookupTransform("camera/0", "green", rospy.Time(0))
			self.x_position_cubes.append(position[0]*0.0006-0.218) 
			self.y_position_cubes.append(position[1]*0.0007-0.1972)
			self.z_position_cubes.append(position[2])

		if self.listener_cubes.canTransform("camera/0", "blue", rospy.Time.now() - rospy.Duration(1.0)) :
			self.visible_cubes.append("B")
			(position,quaternion) = self.listener_cubes.lookupTransform("camera/0","blue", rospy.Time(0))
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
				if self.listener_markers.canTransform("camera/0", full_id, rospy.Time.now() - rospy.Duration(1.0)) :
					self.visible_markers.append(id_marker)
					(position,quaternion) = self.listener_markers.lookupTransform("camera/0", full_id, rospy.Time(0))
					self.prerecorded_x.append(position[0])

			x = np.array(self.prerecorded_x) 
			x = np.argsort(x)

			length = [i for i in range(len(self.prerecorded_x))]
			for k in length:
				self.prerecorded_x_sorted.append(self.prerecorded_x[x[k]])

			print("Done.")




