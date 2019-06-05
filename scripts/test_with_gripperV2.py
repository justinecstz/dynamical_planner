#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import WrenchStamped
import message_filters
import tf
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from robotiq_s_model_control.msg import _SModel_robot_input  as inputMsg
from robotiq_s_model_control.msg import _SModel_robot_output  as outputMsg
import json
from dynamical_planner.pdctrl import PdCtrl

if __name__ == '__main__':
  rospy.init_node('test_with_gripper')
  rospy.sleep(1)
  controller = PdCtrl()
  graphs = plots()

  phase = 1

  rate = rospy.Rate(200)

  grip_command = outputMsg.SModel_robot_output()
  # activate gripper
  grip_command.rACT = 1
  grip_command.rGTO = 1
  grip_command.rSPA = 255
  grip_command.rFRA = 150
  
  # rospy.sleep(2)
  # controller.pubGripper.publish(grip_command) #activate gripper
  # rospy.sleep(5)
  
  go = 0

  while not rospy.is_shutdown():

    controller.time = rospy.Time.now()
    r = rospy.Rate(controller.ctrl_freq)

    if go < 3 :
      controller.timeStart = rospy.get_time()
      go = go + 1

    controller.current_action = "pick1"
    # controller.current_action = "pick2"
    # controller.current_action = "pick3"
    # controller.current_action = "home"

    if controller.current_action == "pick1":
      if controller.position_reached == 0 :
        controller.current_target = controller.joint_positions_p1

    if controller.current_action == "pick2":
      if controller.position_reached == 0 :
        controller.current_target = controller.joint_positions_p2

    if controller.current_action == "pick3":
      if controller.position_reached == 0 :
        controller.current_target = controller.joint_positions_p3

    if controller.current_action == "home":
      if controller.position_reached == 0 :
        controller.current_target = controller.joint_positions_home
      # else:
      #   currentTarget = jointPositionsHome

      # timeBefore = rospy.get_time()
    controller.do_action(controller.current_target)
      # timeAfter = rospy.get_time()
      # totTime = timeAfter - timeBefore
        # print(totTime)
    rate.sleep()