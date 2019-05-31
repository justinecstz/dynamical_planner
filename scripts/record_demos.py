#!/usr/bin/env python
import json
import numpy as np
import roslib
import rospy
from std_msgs.msg import String
from dynamical_planner.world import World
# from sensor_msgs.msg import JointState

class Demo(object):
	def __init__(self):
		self.demonstration = []

	def save_demos(self):
		
		current_demo = []
		go_on = "y"

		while go_on == "y" :
			world.reinitialize()
			ok = raw_input("Please build new state. Press y when you are ready.")
			if ok == "y" :
				#recuperer state depuis la camera par World et append sur current_demo
				#state = str(input("Write state. "))
				world.observe_world()
				world.sort_visible_items()


			if world.listener_markers.canTransform("camera/0", "marker/7", rospy.Time.now() - rospy.Duration(1.0)) :
				world.closed = 1
			else :
				world.closed = 0
			
			world.full_artificial_state()

			print(world.closed)
			confirmation = raw_input("Current state is {0}. Do you want to record it? y/n ".format(world.state_full))
			if confirmation == "y" :
				current_demo.append(world.state_full)

			go_on = raw_input("Do you want to record a new state? y/n ")

		self.demonstration.append(current_demo)
			

	def write_json(self):
		data = {}
		data['demos'] = []
		data['demos'].append({
			'demonstration': self.demonstration
		})
		with open('demonstrations.json', 'w') as outfile:  
			json.dump(data, outfile)

if __name__ == '__main__' :
	rospy.init_node('record_demos')
	demos = Demo()
	world = World()
	wish = raw_input("Do you want to record a new demonstration? y/n ")

	while wish == "y":
		demos.save_demos()
		wish = raw_input("Do you want to record another demonstration? y/n ")

	print("Printing demonstrations in json file.")
	demos.write_json()




	






