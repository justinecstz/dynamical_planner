#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import WrenchStamped
import message_filters
import numpy as np
from robotiq_s_model_control.msg import _SModel_robot_input  as inputMsg
from robotiq_s_model_control.msg import _SModel_robot_output  as outputMsg

jointPositionsP1 = 1*np.array([-0.4339979790845205, 1.3373235520341373, -0.1650918060481264, -0.39354840374588973, 0.16458329147693798, 1.4079477279662593, -1.570306351787338])
jointPositionsP2 = 1*np.array([-0.1526515215095898, 1.186118107041075, -0.4308292301986005, -0.757119167631625, 0.37156025122477176, 1.2697712925938716, -1.5704178650184049])
jointPositionsP3 = 1*np.array([0.008061009099187546, 1.140941414106668, -0.4140705499230182, -0.8524215590582609, 0.40469318154063466, 1.250843827247102, -1.5704030045215718])
jointPositionsHome = 1*np.array([-0.2561309292711379, 0.48341762688080325, -0.07932016919260708, -1.0536090715454243, -0.017271089745345972, 1.5119789493870104, -1.5704001283280475])

positionReached = 0
q_old = jointPositionsHome
gripperCommand = 'o'
count = 0
gripperStatus = 0

def gripper_status(data) :
  global gripperStatus
  gripperStatus = data.gPOA

class PdCtrl:
   def __init__(self):
      rospy.init_node('pd_controller')
      self.ctrl_freq = 200 #achieve the cmd in 0.1 sec

      self.joint_names = ["iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"]

      self.joint_cmd_pub =  rospy.Publisher('/iiwa/PositionController/command', numpy_msg(Float64MultiArray),queue_size=10) #topic important


   def send_cmd(self,command,time_stamp): #command = Float64MultiArray
       next_joint_state = Float64MultiArray()
       next_joint_state.data = command
  
       self.joint_cmd_pub.publish(next_joint_state)

   def lin_ds(self,current_joint_position, target_state):

       global q_star
       global q_send
       global q_speed
       global q_old
      
       alpha = 0.8
       kp = 50 #50
       kd = 0.04 #0.058
       time = 0.02 #0.02

       delta_q = current_joint_position - q_old
       q_old = current_joint_position
       q_star = (1-alpha)*q_star + alpha*target_state
       # q_dot = -kp*(current_joint_position - q_star)
       # q_send = q_send + q_dot*time - kd*(delta_q/time)

       q_acc = kp*(q_star - current_joint_position)-2*np.sqrt(kp)*(delta_q/time)
       q_speed = q_acc*time + q_speed
       q_send = q_acc*time**2 + time*q_speed + q_send   

   def do_action(self,target) :

        global q_star
        global q_send
        global q_speed
        global positionReached
        global currentAction
        global actionPhase
        global count 

        rtol = 1e-2
        atol = 1e-2
        time = rospy.Time.now()
        r = rospy.Rate(controller.ctrl_freq)
        joint_states_msg = rospy.wait_for_message('iiwa/joint_states', numpy_msg(JointState))
        current_position = joint_states_msg.position
        q_star = current_position
        q_send = current_position
        q_speed = current_position*0.005

        controller.lin_ds(current_position, target)
        controller.send_cmd(q_send,time)
   
        if np.allclose(target[2:7],current_position[2:7], rtol, atol) and ((currentTarget==jointPositionsP1).all() or \
              (currentTarget==jointPositionsP2).all() or (currentTarget==jointPositionsP3).all()) :
          
          positionReached = 1
          command = outputMsg.SModel_robot_output()
          # close gripper
          # command.rPRA = 255
          # pubGripper.publish(command)
          # gripper_msg = rospy.wait_for_message('SModelRobotInput', inputMsg.SModel_robot_input)
          # while gripper_msg.gPOA < 200 :
          #    gripper_msg = rospy.wait_for_message('SModelRobotInput', inputMsg.SModel_robot_input)
             # print(gripper_msg.gPOA)
        elif positionReached == 1 and np.allclose(jointPositionsHome[2:7],current_position[2:7], rtol, atol)  :
          positionReached = 0
          count = 1

        r.sleep()


if __name__ == '__main__':
    controller = PdCtrl()
    rate = rospy.Rate(200)
   
    pubGripper = rospy.Publisher('SModelRobotOutput', outputMsg.SModel_robot_output, queue_size = 1);

    command = outputMsg.SModel_robot_output()
    # activate gripper
    command.rACT = 1
    command.rGTO = 1
    command.rSPA = 255
    command.rFRA = 150
    
    # rospy.sleep(2)
    # pubGripper.publish(command) #activate gripper
    # rospy.sleep(5)
    

    while not rospy.is_shutdown():
        time = rospy.Time.now()
        r = rospy.Rate(controller.ctrl_freq)

        if count == 0:
          currentAction= "pick1"

          if currentAction == "pick1":
            if positionReached == 0 :
              currentTarget = jointPositionsP1
              gripperCommand = 'c'
            else:
              currentTarget = jointPositionsHome

          controller.do_action(currentTarget)
          rate.sleep()
