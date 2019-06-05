#!/usr/bin/env python
import roslib
import rospy
import tf
import json
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rospy.numpy_msg import numpy_msg
import numpy as np
from dynamical_planner.gripper import Gripper
from srv import ControlGripper
from srv import ControlGripperRequest
from srv import ControlGripperResponse

class PdCtrl:
	def __init__(self):
		# Real robot new positions
		# self.joint_positions_p1 = 1*np.array([0.313771815355006, 0.7874442301146876, -0.35008257875556276, -1.605427980371526, 0.39355274195809514, 0.7924178032543101, -1.4842731591265332])
		# self.joint_positions_p2 = 1*np.array([0.5628616530365289, 0.8266034640114479, -0.343280991826965, -1.548290132212003, 0.36961525944794194, 0.787609672383395, -1.4840846472048561])
		# self.joint_positions_p3 = 1*np.array([0.7087980537375754, 0.9364973682354464, -0.3175523589760338, -1.3048212555591447, 0.32881175402628704, 0.9519498877320673, -1.483692343658399])
		# self.joint_positions_home = 1*np.array([0.5569787767738359, 0.2807509613788765, -0.2521163338340084, -1.4935594952802898, 0.06308649381713503, 1.3422390616692874, -1.528839315708223])
		
		# FOR VIDEOS WITH SIMULATION
		# self.joint_positions_p1 = 1*np.array([0.313771815355006, 0.7874442301146876, -0.35008257875556276, -1.605427980371526, 0.39355274195809514, 0.7924178032543101, -1.4842731591265332])
		# self.joint_positions_p2 = 1*np.array([0.5628616530365289, 0.8266034640114479, -0.343280991826965, -1.548290132212003, 0.36961525944794194, 0.787609672383395, -1.4840846472048561])
		# self.joint_positions_p3 = 1*np.array([0.9087980537375754, 0.9364973682354464, -0.3175523589760338, -1.3048212555591447, 0.32881175402628704, 0.9519498877320673, -1.483692343658399])
		# self.joint_positions_home = 1*np.array([0.5569787767738359, 0.2807509613788765, -0.2521163338340084, -1.4935594952802898, 0.06308649381713503, 1.3422390616692874, -1.528839315708223])

		# self.joint_positions_home = 1*np.array([0.5569787767738359, 0.2807509613788765, -0.2521163338340084, -1.4935594952802898, 0.06308649381713503, 1.3422390616692874, -1.528839315708223])
		# self.joint_positions_p1 = 1*np.array([0.27099370506783615, 0.7950941203635832, -0.18290599687813244, -1.545624301259042, 0.19252302584488534, 0.8620693993694969, -0.023598616669335572])
		# self.joint_positions_p2 = 1*np.array([0.6446756195683863, 0.9211181322779567, -0.4819512971273034, -1.4284985165235762, 0.5240945070507964, 0.9734429360283877, 0.024398803334734404])
		# self.joint_positions_p3 = 1*np.array([1.0506115772018698, 1.2189853831695028, -1.0281253363138159, -1.1760928235711319, 0.9755587628795142, 1.2917956014430403, 0.11816475720439637])
		# self.joint_positions_handover = 1*np.array([0.5569787767738359, 0.2807509613788765, -0.2521163338340084, -1.4935594952802898, 0.06308649381713503, 0.2022390616692874, -1.528839315708223])
		
		self.joint_positions_home = 1*np.array([0.20024075874350197, 0.3457841368224398, -0.39772640410707416, -1.5429807609967892, 0.1143476257008436, 1.2734894582565162, -5.7464311123443805e-05])
		self.joint_positions_p1 = 1*np.array([-0.24520071701864543, 0.8216742323795524, -0.2692404729649888, -1.497127918129015, 0.3290061859738872, 0.875714937494543, 0.027934688991962303])
		self.joint_positions_p2 = 1*np.array([0.005690829067532814, 0.7855678600152904, -0.21038864090325585, -1.56908060519925, 0.14593324885458367, 0.8493683380970197, -0.2437001721577387])
		self.joint_positions_p3 = 1*np.array([0.15672118448444747, 0.7813835079646257, -0.12592376380188047, -1.560340569973171, 0.14926112433896319, 0.8510714761960086, -0.025095686034286177])


		self.pub = rospy.Publisher('action_state', String, queue_size = 10)
		# self.pub_for_gripper = rospy.Publisher('gripper_command', String, queue_size = 10)
		self.ctrl_freq = 200
		self.q_send = 1*np.array([-0.2561309292711379, 0.48341762688080325, -0.07932016919260708, -1.0536090715454243, -0.017271089745345972, 1.5119789493870104, -1.5704001283280475])
		self.q_old = 1*np.array([-0.2561309292711379, 0.48341762688080325, -0.07932016919260708, -1.0536090715454243, -0.017271089745345972, 1.5119789493870104, -1.5704001283280475])
		self.old_q_send = 1*np.array([-0.2561309292711379, 0.48341762688080325, -0.07932016919260708, -1.0536090715454243, -0.017271089745345972, 1.5119789493870104, -1.5704001283280475])
		self.joint_names = ["iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"]
		self.joint_cmd_pub =  rospy.Publisher('/iiwa/PositionController/command', numpy_msg(Float64MultiArray),queue_size=10)
		self.joint_state_sub = rospy.Subscriber('iiwa/joint_states', numpy_msg(JointState), self.get_joint_state)
		self.joint_current_states = self.joint_positions_home 
		self.position_reached = 0
		self.current_action = "home"
		self.current_target = self.joint_positions_home
		self.gripper_service = rospy.ServiceProxy('gripper_command',ControlGripper)


		#PLOTS
		self.commandJoints = [[] for _ in np.arange(7)]
		self.errorJoints = [[] for _ in np.arange(7)]
		self.speedJoints = [[] for _ in np.arange(7)]
		self.endEffxyz = [[] for _ in np.arange(3)]
		self.timeStart = 0
		self.timeVector = []
		self.listenerEndEff = tf.TransformListener()
		self.time = []

		self.subplotID = [331,332,333,334,335,336,337]
		self.titles = ['Joint 1','Joint 2','Joint 3','Joint 4','Joint 5','Joint 6','Joint 7']

	def save_data(self,command,error,speed,time,endEffPos) :

		for i in np.arange(7):
			self.commandJoints[i].append(command[i])
			self.errorJoints[i].append(error[i])
			self.speedJoints[i].append(speed[i])

		for i in np.arange(3):
			self.endEffxyz[i].append(endEffPos[i])

		timeSim = time - self.timeStart
		self.timeVector.append(timeSim)

	def read_data(self,fileName,topic,dataList):
		with open(fileName) as json_file:  
			data = json.load(json_file)
			for p in data[topic]:
				for i in np.arange(7):
					dataList[i].append(float(p['Joint' + str(i)]))


	def write_json(self,command,error,speed,time,endEffPos):

		dataCmdJ = {}
		dataCmdJ['commandJoints'] = []
		dataCmdJ['commandJoints'].append({
			'Joint0' : command[0],
			'Joint1' : command[1],
			'Joint2' : command[2],
			'Joint3' : command[3],
			'Joint4' : command[4],
			'Joint5' : command[5],
			'Joint6' : command[6]
		})
		with open('commandJoints.json', 'w') as outfile:  
			json.dump(dataCmdJ, outfile)

		dataError = {}
		dataError['errorJoints'] = []
		dataError['errorJoints'].append({
			'Joint0' : error[0],
			'Joint1' : error[1],
			'Joint2' : error[2],
			'Joint3' : error[3],
			'Joint4' : error[4],
			'Joint5' : error[5],
			'Joint6' : error[6]
		})
		with open('errorJoints.json', 'w') as outfile:  
			json.dump(dataError, outfile)

		dataSpeed = {}
		dataSpeed['speedJoints'] = []
		dataSpeed['speedJoints'].append({
			'Joint0' : speed[0],
			'Joint1' : speed[1],
			'Joint2' : speed[2],
			'Joint3' : speed[3],
			'Joint4' : speed[4],
			'Joint5' : speed[5],
			'Joint6' : speed[6]
		})
		with open('speedJoints.json', 'w') as outfile:  
			json.dump(dataSpeed, outfile)

		dataEndEff = {}
		dataEndEff['endEffPos'] = []
		dataEndEff['endEffPos'].append({
			'x' : endEffPos[0],
			'y' : endEffPos[1],
			'z' : endEffPos[2]
		})
		with open('endEff.json', 'w') as outfile:  
			json.dump(dataEndEff, outfile)

		dataTime = {}
		dataTime['time'] = []
		for i in np.arange(len(self.timeVector)):
			dataTime['time'].append({
				'timeSec' : self.timeVector[i]
			})
		with open('time.json', 'w') as outfile:  
			json.dump(dataTime, outfile)

	def get_joint_state(self,data):
		self.joint_current_states = data.position

	def send_cmd(self,command): #command = Float64MultiArray
		next_joint_state = Float64MultiArray()

		next_joint_state.data = command 
		self.joint_cmd_pub.publish(next_joint_state)

	def lin_ds(self,current_joint_position, target_state):
		time = 0.02 #0.02
 		K = 15*np.array([1,1,1,1,1,1,0.3])#15
 		alpha = 0.8

		delta_q = current_joint_position - self.q_old
		self.q_old = current_joint_position

		q_speed_max = [0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.0001]
		error = current_joint_position-target_state  

		q_dot = -K*error

		for i in np.arange(7):
			if abs(q_dot[i]) > q_speed_max[i]:
				q_dot[i] = np.sign(q_dot[i])*q_speed_max[i]

		self.q_send = q_dot*time + self.q_old
		timeSim = rospy.get_time()
		self.listenerEndEff.waitForTransform('iiwa_link_ee','world', rospy.Time(), rospy.Duration(1))
		(endEffPos,endEffQuat) = self.listenerEndEff.lookupTransform('world','iiwa_link_ee',rospy.Time(0))
		self.save_data(self.q_send,error,q_dot,timeSim,endEffPos)
		# self.q_send = alpha*(q_dot*time + self.q_old)+(1-alpha)*self.old_q_send
		# self.old_q_send = self.q_send

	def do_action(self,target) :

		tol = 3e-2

		r = rospy.Rate(self.ctrl_freq)
		
 		current_position = self.joint_current_states
		self.q_send = current_position
		self.lin_ds(current_position, target)
		self.send_cmd(self.q_send)

		diff_target = abs(target[0:6]-current_position[0:6])
		diff_home = abs(self.joint_positions_home[0:6]-current_position[0:6])

		action_state = "not terminated"
		self.pub.publish(action_state)

		# print(np.amax(diff_target) < tol)
		if np.amax(diff_target) < tol and ((self.current_target==self.joint_positions_p1).all() or \
					(self.current_target==self.joint_positions_p2).all() or (self.current_target==self.joint_positions_p3).all()) :

			# print('hello')
			self.position_reached = 1
			self.pub.publish(action_state)
			self.write_json(self.commandJoints,self.errorJoints,self.speedJoints,self.timeVector,self.endEffxyz)

			if self.current_action == "pick1" or self.current_action == "pick2" or self.current_action == "pick3": #if self.current_action[:-1] == "pick":
				print("Closing gripper")
				msg = ControlGripperRequest()
				msg.command = ControlGripperRequest.CLOSE
				# print(msg.command)
				# res = self.gripper_service(msg)
				# self.gripper_service("c")
				# self.pub_for_gripper.publish("c")


			elif self.current_action == "place1" or self.current_action == "place2" or self.current_action == "place3":
				print("Opening gripper")
				# print('hello2')
				# self.gripper_service("o")
				msg = ControlGripperRequest()
				msg.command = ControlGripperRequest.OPEN
				# res = self.gripper_service(msg)
				# self.pub_for_gripper.publish("o")
	

		elif np.amax(diff_home) < tol and (self.current_target==self.joint_positions_home).all() :
				self.position_reached = 0
				action_state = "terminated"
				self.pub.publish(action_state)
				self.write_json(self.commandJoints,self.errorJoints,self.speedJoints,self.timeVector,self.endEffxyz)

		r.sleep()