#!/usr/bin/env python
import json
import numpy as np
import roslib
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class Pos(object):
	def __init__(self):
		self.currentPosition = 1*np.array([0, 0, 0, 0, 0, 0, 0])
		self.joint_state_sub = rospy.Subscriber('iiwa/joint_states', numpy_msg(JointState), self.get_joint_state)
		self.p1 = 1*np.array([0.313771815355006, 0.7874442301146876, -0.35008257875556276, -1.605427980371526, 0.39355274195809514, 0.7924178032543101, -1.4842731591265332])
		self.p2 = 1*np.array([0.5628616530365289, 0.8266034640114479, -0.343280991826965, -1.548290132212003, 0.36961525944794194, 0.787609672383395, -1.4840846472048561])
		self.p3 = 1*np.array([0.7087980537375754, 0.9364973682354464, -0.3175523589760338, -1.3048212555591447, 0.32881175402628704, 0.9519498877320673, -1.483692343658399])
		self.pHome = 1*np.array([0.5569787767738359, 0.2807509613788765, -0.2521163338340084, -1.4935594952802898, 0.06308649381713503, 1.3422390616692874, -1.528839315708223])

	def get_joint_state(self,data):
		self.currentPosition = data.position

	def save_pos(self):
		
		positionID = input("Enter position ID (1, 2, 3 or h). ")
		confirmation = input("Do you want to record position {0}? y/n ".format(positionID))

		if confirmation == "y":
			if positionID == "1":
				pos.p1 = pos.currentPosition
			elif positionID == "2":
				pos.p2 = pos.currentPosition
			elif positionID == "3":
				pos.p3 = pos.currentPosition
			elif positionID == "h":
				pos.pHome = pos.currentPosition

def write_json(p1,p2,p3,pHome):
	data = {}
	data['positions'] = []
	data['positions'].append({
		'p1' : p1,
		'p2' : p2,
		'p3' : p3,
		'pHome' : pHome,
	})
	with open('positions.json', 'w') as outfile:  
		json.dump(data, outfile)

if __name__ == '__main__' :
	rospy.init_node('record_positions')
	pos = Pos()

	wish = input("Please bring robot's end effector to desired target. Press y when you are ready. ")

	while wish == "y":
		pos.save_pos()
		wish = input("Do you want to record another position? y/n ")

	print("Printing position in json file.")
	p1 = pos.p1.tolist()
	p2 = pos.p2.tolist()
	p3 = pos.p3.tolist()
	pHome = pos.pHome.tolist()
	write_json(p1,p2,p3,pHome)




	






