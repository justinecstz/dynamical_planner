#!/usr/bin/env python
import roslib
import rospy
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
		self.joint_positions_p1 = 1*np.array([0.313771815355006, 0.7874442301146876, -0.35008257875556276, -1.605427980371526, 0.39355274195809514, 0.7924178032543101, -1.4842731591265332])
		self.joint_positions_p2 = 1*np.array([0.5628616530365289, 0.8266034640114479, -0.343280991826965, -1.548290132212003, 0.36961525944794194, 0.787609672383395, -1.4840846472048561])
		self.joint_positions_p3 = 1*np.array([0.9087980537375754, 0.9364973682354464, -0.3175523589760338, -1.3048212555591447, 0.32881175402628704, 0.9519498877320673, -1.483692343658399])
		self.joint_positions_home = 1*np.array([0.5569787767738359, 0.2807509613788765, -0.2521163338340084, -1.4935594952802898, 0.06308649381713503, 1.3422390616692874, -1.528839315708223])

		self.joint_positions_handover = 1*np.array([0.5569787767738359, 0.2807509613788765, -0.2521163338340084, -1.4935594952802898, 0.06308649381713503, 0.2022390616692874, -1.528839315708223])
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

	def get_joint_state(self,data):
		self.joint_current_states = data.position

	def send_cmd(self,command): #command = Float64MultiArray
		next_joint_state = Float64MultiArray()

		next_joint_state.data = command 
		self.joint_cmd_pub.publish(next_joint_state)

	def lin_ds(self,current_joint_position, target_state):
		time = 0.02 #0.02
 		K = 0.3*np.array([1,1,1,1,1,1,0.3])#10
 		alpha = 0.8

		delta_q = current_joint_position - self.q_old
		self.q_old = current_joint_position

		q_speed_max = [0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.001]
		error = current_joint_position-target_state  

		q_dot = -K*error

		for i in np.arange(7):
			if abs(q_dot[i]) > q_speed_max[i]:
				q_dot[i] = np.sign(q_dot[i])*q_speed_max[i]

		#self.q_send = q_dot*time + self.q_old
		self.q_send = alpha*(q_dot*time + self.q_old)+(1-alpha)*self.old_q_send
		self.old_q_send = self.q_send

	def do_action(self,target) :

		rtol = 1e-4
		atol = 1e-4
		tol = 5e-2

		r = rospy.Rate(self.ctrl_freq)
		
 		current_position = self.joint_current_states
		self.q_send = current_position
		self.lin_ds(current_position, target)
		self.send_cmd(self.q_send)

		diff_target = abs(target[0:6]-current_position[0:6])
		diff_home = abs(self.joint_positions_home[0:6]-current_position[0:6])

		action_state = "not terminated"
		self.pub.publish(action_state)

		if np.amax(diff_target) < tol and ((self.current_target==self.joint_positions_p1).all() or \
					(self.current_target==self.joint_positions_p2).all() or (self.current_target==self.joint_positions_p3).all()) :

			self.position_reached = 1
			self.pub.publish(action_state)

			if self.current_action == "pick1" or self.current_action == "pick2" or self.current_action == "pick3":
				print("Closing gripper")
				msg = ControlGripperRequest()
				msg.command = ControlGripperRequest.CLOSE
				res = self.gripper_service(msg)
				# self.gripper_service("c")
				# self.pub_for_gripper.publish("c")


			elif self.current_action == "place1" or self.current_action == "place2" or self.current_action == "place3":
				print("Opening gripper")
				# self.gripper_service("o")
				msg = ControlGripperRequest()
				msg.command = ControlGripperRequest.OPEN
				res = self.gripper_service(msg)
				# self.pub_for_gripper.publish("o")
	

		elif np.amax(diff_home) < tol and (self.current_target==self.joint_positions_home).all() :
				self.position_reached = 0
				action_state = "terminated"
				self.pub.publish(action_state)

		r.sleep()