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

# jointPositionsP1 = 1*np.array([-0.4339979790845205, 1.3373235520341373, -0.1650918060481264, -0.39354840374588973, 0.16458329147693798, 1.4079477279662593, -1.570306351787338])
# jointPositionsP2 = 1*np.array([-0.1526515215095898, 1.186118107041075, -0.4308292301986005, -0.757119167631625, 0.37156025122477176, 1.2697712925938716, -1.5704178650184049])
# jointPositionsP3 = 1*np.array([0.008061009099187546, 1.140941414106668, -0.4140705499230182, -0.8524215590582609, 0.40469318154063466, 1.250843827247102, -1.5704030045215718])
# jointPositionsHome = 1*np.array([-0.2561309292711379, 0.48341762688080325, -0.07932016919260708, -1.0536090715454243, -0.017271089745345972, 1.5119789493870104, -1.5704001283280475])

jointPositionsP1 = 1*np.array([0.313771815355006, 0.7874442301146876, -0.35008257875556276, -1.605427980371526, 0.39355274195809514, 0.7924178032543101, -1.4842731591265332])
jointPositionsP2 = 1*np.array([0.5628616530365289, 0.8266034640114479, -0.343280991826965, -1.548290132212003, 0.36961525944794194, 0.787609672383395, -1.4840846472048561])
jointPositionsP3 = 1*np.array([0.7087980537375754, 0.9364973682354464, -0.3175523589760338, -1.3048212555591447, 0.32881175402628704, 0.9519498877320673, -1.483692343658399])
jointPositionsHome = 1*np.array([0.5569787767738359, 0.2807509613788765, -0.2521163338340084, -1.4935594952802898, 0.06308649381713503, 1.3422390616692874, -1.528839315708223])

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

    # if self.write_count == 0:
    #   dataCmdJ = {}
    #   dataCmdJ['commandJoints'] = []
    #   dataCmdJ['commandJoints'].append({
    #      'Joint0' : str(command[0]),
    #      'Joint1' : str(command[1]),
    #      'Joint2' : str(command[2]),
    #      'Joint3' : str(command[3]),
    #      'Joint4' : str(command[4]),
    #      'Joint5' : str(command[5]),
    #      'Joint6' : str(command[6])
    #   })
    #   with open('commandJoints.txt', 'w') as outfile:  
    #     json.dump(dataCmdJ, outfile)

    # else:
    #   dataCmdJ = {}
    #   dataCmdJ['commandJoints'] = []
    #   self.read_data('commandJoints.txt','commandJoints',self.commandJoints)

    #   for i in np.arange(7):
    #     self.commandJoints[i].append(command[i])

    #   for j in np.arange(len(self.commandJoints[0])):
    #     dataCmdJ['commandJoints'].append({
    #      'Joint0' : str(self.commandJoints[0][j]),
    #      'Joint1' : str(self.commandJoints[1][j]),
    #      'Joint2' : str(self.commandJoints[2][j]),
    #      'Joint3' : str(self.commandJoints[3][j]),
    #      'Joint4' : str(self.commandJoints[4][j]),
    #      'Joint5' : str(self.commandJoints[5][j]),
    #      'Joint6' : str(self.commandJoints[6][j])
    #     })
    #     with open('commandJoints.txt', 'w') as outfile:  
    #       json.dump(dataCmdJ, outfile)



  def read_data(self,fileName,topic,dataList):
    with open(fileName) as json_file:  
      data = json.load(json_file)
      for p in data[topic]:
        for i in np.arange(7):
          dataList[i].append(float(p['Joint' + str(i)]))


  def write_json(self,command,error,speed,time,endEffPos):

    # print(type(command[0][0]))
    # print(command[0])
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
      self.joint_state_sub = rospy.Subscriber('iiwa/joint_states', numpy_msg(JointState), self.get_joint_state)
      self.pubGripper = rospy.Publisher('SModelRobotOutput', outputMsg.SModel_robot_output, queue_size = 1);
      self.jointCurrentStates = jointPositionsHome 

   def get_joint_state(self,data):
      self.jointCurrentStates = data.position

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
       kd = 0.04 
       time = 0.02 #0.02
       K = 11 #10

       delta_q = current_joint_position - self.q_old
       self.q_old = current_joint_position

       q_speed_max = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.01]

       # q_star = (1-alpha)*q_star + alpha*target_state

       # q_acc = kp*(q_star - current_joint_position)-2*np.sqrt(kp)*(delta_q/time)
       # q_speed = q_acc*time + q_speed
       # q_send = q_acc*time**2 + time*q_speed + q_send
       error = current_joint_position-target_state  

       #Verifier ici que q_dot (en valeur abs ?) ne depasse pas les limites, pour chaque joint. Si oui, mettre egal a q_dot_max

       q_dot = -K*error

       for i in np.arange(7):
        if q_dot[i] > q_speed_max[i]:
          q_dot[i] = q_speed_max[i]

       self.q_send = q_dot*time + self.q_old

       graphs.listenerEndEff.waitForTransform('iiwa_link_ee','world', rospy.Time(), rospy.Duration(1))
       (endEffPos,endEffQuat) = graphs.listenerEndEff.lookupTransform('world','iiwa_link_ee',rospy.Time(0))

       timeSim = rospy.get_time()
       # if test == 0:
       graphs.save_data(self.q_send,error,q_dot,timeSim,endEffPos)
       # graphs.write_count = 1
        # test = 1

   def do_action(self,target) :

        # global q_star
        # global q_send
        # global q_speed
        global positionReached
        global currentAction
        # global actionPhase
        global count 

        rtol = 1e-3 #1e-3
        atol = 1e-3 #1e-3
        tol = 5e-3
        r = rospy.Rate(controller.ctrl_freq)
        # timeBefore = rospy.get_time()
        # joint_states_msg = rospy.wait_for_message('iiwa/joint_states', numpy_msg(JointState))
        # timeAfter = rospy.get_time()
        # totTime = timeAfter - timeBefore
        # print(totTime)
        # current_position = joint_states_msg.position
        # q_star = current_position

        current_position = self.jointCurrentStates
        self.q_send = current_position
        # q_speed = current_position*0.005

        self.lin_ds(current_position, target)
        self.send_cmd(self.q_send)

        # print(np.linalg.norm(target-current_position))
        
        diff = abs(target-current_position)
        # print(diff)
        # if np.linalg.norm(target-current_position) < tol:
        if np.all(diff < tol):
            # if np.allclose(target[2:7],current_position[2:7], rtol, atol) and ((currentTarget==jointPositionsP1).all() or \
            #   (currentTarget==jointPositionsP2).all() or (currentTarget==jointPositionsP3).all()) :
          
            positionReached = 1
            # grip_command = outputMsg.SModel_robot_output()
            # close gripper
            grip_command.rACT = 1
            grip_command.rGTO = 1
            grip_command.rSPA = 255
            grip_command.rFRA = 150
            grip_command.rPRA = 255

            graphs.write_json(graphs.commandJoints,graphs.errorJoints,graphs.speedJoints,graphs.timeVector,graphs.endEffxyz)
            
            # graphs.plot_endeff()
            # graphs.plot_cmd_error()
            # graphs.plot_speed_error()
            # graphs.plot_speed_time()
            # graphs.plot_command_time()

            # self.pubGripper.publish(grip_command)
            # gripper_msg = rospy.wait_for_message('SModelRobotInput', inputMsg.SModel_robot_input)
            # while gripper_msg.gPOA < 200 :
            #    gripper_msg = rospy.wait_for_message('SModelRobotInput', inputMsg.SModel_robot_input)
               # print(gripper_msg.gPOA)
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
  
  ##############
  go = 0
  ##############

  while not rospy.is_shutdown():

    time = rospy.Time.now()
    r = rospy.Rate(controller.ctrl_freq)

    if go < 3 :
      graphs.timeStart = rospy.get_time()
      go = go + 1

    # if count == 0:
    # if phase == 1:
    #   currentTarget = jointPositionsHome
    #   controller.do_action(currentTarget)
    #   if np.allclose(controller.jointCurrentStates,jointPositionsHome,1e-3,1e-3):
    #     phase = 2
    # else:

    currentAction= "pick1"
    # currentAction= "home"

    if currentAction == "pick1":
      if positionReached == 0 :
        currentTarget = jointPositionsP1

    if currentAction == "home":
      if positionReached == 0 :
        currentTarget = jointPositionsHome
      # else:
      #   currentTarget = jointPositionsHome

      # timeBefore = rospy.get_time()
    controller.do_action(currentTarget)
      # timeAfter = rospy.get_time()
      # totTime = timeAfter - timeBefore
        # print(totTime)
    rate.sleep()