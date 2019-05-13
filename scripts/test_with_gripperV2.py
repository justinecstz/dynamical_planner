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

jointPositionsP1 = 1*np.array([-0.4339979790845205, 1.3373235520341373, -0.1650918060481264, -0.39354840374588973, 0.16458329147693798, 1.4079477279662593, -1.570306351787338])
jointPositionsP2 = 1*np.array([-0.1526515215095898, 1.186118107041075, -0.4308292301986005, -0.757119167631625, 0.37156025122477176, 1.2697712925938716, -1.5704178650184049])
jointPositionsP3 = 1*np.array([0.008061009099187546, 1.140941414106668, -0.4140705499230182, -0.8524215590582609, 0.40469318154063466, 1.250843827247102, -1.5704030045215718])
jointPositionsHome = 1*np.array([-0.2561309292711379, 0.48341762688080325, -0.07932016919260708, -1.0536090715454243, -0.017271089745345972, 1.5119789493870104, -1.5704001283280475])

positionReached = 0
q_old = jointPositionsHome
gripperCommand = 'o'
count = 0
gripperStatus = 0
# test = 0

#https://stackoverflow.com/questions/8713620/appending-items-to-a-list-of-lists-in-python

def gripper_status(data) :
  global gripperStatus
  gripperStatus = data.gPOA

class plots(object):
  def __init__(self):
    self.commandJoints = [[] for _ in np.arange(7)]
    self.errorJoints = [[] for _ in np.arange(7)]
    self.speedJoints = [[] for _ in np.arange(7)]
    self.endEffxyz = [[] for _ in np.arange(3)]
    self.timeStart = 0
    self.timeVector = []

    self.listenerEndEff = tf.TransformListener()

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

  def plot_cmd_error(self):

    for i in np.arange(7):
      plt.subplot(self.subplotID[i])
      plt.plot(self.errorJoints[i],self.commandJoints[i],0,jointPositionsP1[i],'r+')
      plt.xlabel('Joint error')
      plt.ylabel('Joint command')
      plt.title(self.titles[i])

    plt.show()

  def plot_speed_error(self):

    for i in np.arange(7):
      plt.subplot(self.subplotID[i])
      plt.plot(self.errorJoints[i],self.speedJoints[i])
      plt.xlabel('Joint error')
      plt.ylabel('Joint speed')
      plt.title(self.titles[i])

    plt.show()

  def plot_speed_time(self):

    self.timeVector[0] = 0

    for i in np.arange(7):
      plt.subplot(self.subplotID[i])
      plt.plot(self.timeVector,self.speedJoints[i])
      plt.xlabel('Time [s]')
      plt.ylabel('Joint speed')
      plt.title(self.titles[i])

    plt.show()

  def plot_command_time(self):

    self.timeVector[0] = 0

    for i in np.arange(7):
      plt.subplot(self.subplotID[i])
      plt.plot(self.timeVector,self.commandJoints[i])
      plt.xlabel('Time [s]')
      plt.ylabel('Joint command')
      plt.title(self.titles[i])

    plt.show()

  def plot_endeff(self):

    ax = plt.axes(projection='3d')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.plot3D(self.endEffxyz[0],self.endEffxyz[1],self.endEffxyz[2])
    plt.title('End effector trajectory')
    plt.show()

    plt.subplot(131)
    plt.plot(self.endEffxyz[0],self.endEffxyz[1])
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('End effector trajectory x-y plane')

    plt.subplot(132)
    plt.plot(self.endEffxyz[0],self.endEffxyz[2])
    plt.xlabel('x')
    plt.ylabel('z')
    plt.title('End effector trajectory x-z plane')

    plt.subplot(133)
    plt.plot(self.endEffxyz[1],self.endEffxyz[2])
    plt.xlabel('y')
    plt.ylabel('z')
    plt.title('End effector trajectory y-z plane')

    plt.show()

class PdCtrl(object):
   def __init__(self):
      self.ctrl_freq = 200 #achieve the cmd in 0.1 sec
      self.q_send = 1*np.array([-0.2561309292711379, 0.48341762688080325, -0.07932016919260708, -1.0536090715454243, -0.017271089745345972, 1.5119789493870104, -1.5704001283280475])
      self.q_old = 1*np.array([-0.2561309292711379, 0.48341762688080325, -0.07932016919260708, -1.0536090715454243, -0.017271089745345972, 1.5119789493870104, -1.5704001283280475])
      # self.q_speed = 1*np.array([0, 0, 0, 0, 0, 0, 0])
      self.joint_names = ["iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"]
      self.joint_cmd_pub =  rospy.Publisher('/iiwa/PositionController/command', numpy_msg(Float64MultiArray),queue_size=10) #topic important
      self.pubGripper = rospy.Publisher('SModelRobotOutput', outputMsg.SModel_robot_output, queue_size = 1);

   def send_cmd(self, joint_command): #command = Float64MultiArray
       next_joint_state = Float64MultiArray()
       next_joint_state.data = joint_command
  
       self.joint_cmd_pub.publish(next_joint_state)

   def lin_ds(self,current_joint_position, target_state):

       global q_star
       global test
       # global q_send
       # global q_speed
       # global q_old
      
       alpha = 0.8
       kp = 50 #50
       kd = 0.04 #0.058
       time = 0.02 #0.02
       K = 0.92

       delta_q = current_joint_position - self.q_old
       self.q_old = current_joint_position
       # q_star = (1-alpha)*q_star + alpha*target_state

       # q_acc = kp*(q_star - current_joint_position)-2*np.sqrt(kp)*(delta_q/time)
       # q_speed = q_acc*time + q_speed
       # q_send = q_acc*time**2 + time*q_speed + q_send
       error = current_joint_position-target_state  

       #Verifier ici que q_dot (en valeur abs ?) ne depasse pas les limites, pour chaque joint. Si oui, mettre egal a q_dot_max
       q_dot = -K*error
       self.q_send = q_dot*time + self.q_old

       graphs.listenerEndEff.waitForTransform('iiwa_link_ee','world', rospy.Time(), rospy.Duration(1))
       (endEffPos,endEffQuat) = graphs.listenerEndEff.lookupTransform('iiwa_link_ee','world',rospy.Time(0))

       timeSim = rospy.get_time()
       # if test == 0:
       graphs.save_data(self.q_send,error,q_dot,timeSim,endEffPos)
        # test = 1

   def do_action(self,target) :

        # global q_star
        # global q_send
        # global q_speed
        global positionReached
        global currentAction
        # global actionPhase
        global count 

        rtol = 1e-2
        atol = 1e-2
        r = rospy.Rate(controller.ctrl_freq)
        joint_states_msg = rospy.wait_for_message('iiwa/joint_states', numpy_msg(JointState))

        current_position = joint_states_msg.position
        # q_star = current_position
        self.q_send = current_position
        # q_speed = current_position*0.005

        self.lin_ds(current_position, target)
        self.send_cmd(self.q_send)
   
        if np.allclose(target[2:7],current_position[2:7], rtol, atol) and ((currentTarget==jointPositionsP1).all() or \
              (currentTarget==jointPositionsP2).all() or (currentTarget==jointPositionsP3).all()) :
          
            positionReached = 1
            grip_command = outputMsg.SModel_robot_output()
            # close gripper
            grip_command.rACT = 1
            grip_command.rGTO = 1
            grip_command.rSPA = 255
            grip_command.rFRA = 150
            grip_command.rPRA = 255
            
            graphs.plot_endeff()
            graphs.plot_cmd_error()
            graphs.plot_speed_error()
            graphs.plot_speed_time()
            graphs.plot_command_time()

            # self.pubGripper.publish(command)
            # gripper_msg = rospy.wait_for_message('SModelRobotInput', inputMsg.SModel_robot_input)
            # while gripper_msg.gPOA < 200 :
            #    gripper_msg = rospy.wait_for_message('SModelRobotInput', inputMsg.SModel_robot_input)
            #    print(gripper_msg.gPOA)
        # elif positionReached == 1 and np.allclose(jointPositionsHome[2:7],current_position[2:7], rtol, atol)  :
        #   positionReached = 0
        #   count = 1
          ################
          #Essayer de plotter les 4 plots pour chaque transition Home-P et inversement
        
        # graphs.plot_speed_error()
        # graphs.plot_speed_time()
        # graphs.plot_command_time()
          ################ 
        r.sleep()


if __name__ == '__main__':
  rospy.init_node('test_with_gripper')
  rospy.sleep(1)
  controller = PdCtrl()
  graphs = plots()

  rate = rospy.Rate(200)

  grip_command = outputMsg.SModel_robot_output()
  # activate gripper
  grip_command.rACT = 1
  grip_command.rGTO = 1
  grip_command.rSPA = 255
  grip_command.rFRA = 150
  
  # rospy.sleep(2)
  # controller.pubGripper.publish(command) #activate gripper
  # rospy.sleep(5)
  
  ##############
  go = 0
  ##############

  while not rospy.is_shutdown():

    time = rospy.Time.now()
    r = rospy.Rate(controller.ctrl_freq)

    if go < 3 :
      graphs.timeStart = rospy.get_time()
      go = go + 1

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