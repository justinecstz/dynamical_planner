#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import String

if __name__ == '__main__' :
	gripper = Gripper()
	command = "c"
	# command = "o"

	pub = rospy.Publisher('gripper_command', String, self.do_gripper_command)
	pub.publish(command)


