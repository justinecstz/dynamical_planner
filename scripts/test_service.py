#!/usr/bin/env python
import roslib
import rospy
from dynamical_planner.srv import ControlGripper
from dynamical_planner.srv import ControlGripperResponse
from dynamical_planner.srv import ControlGripperRequest
from dynamical_planner.gripper import Gripper

if __name__ == '__main__' :
	rospy.init_node('testGripper')
	gripper_service = rospy.ServiceProxy('gripper_command',ControlGripper)
	gripper = Gripper()

	msg = ControlGripperRequest()
	msg.command = ControlGripperRequest.OPEN
	res = self.gripper_service(msg)

	print("Done.")