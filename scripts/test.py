#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import WrenchStamped
import message_filters
import numpy as np

jointPositionsP1 = 1*np.array([-0.044405747733462106, 1.19814689840694254, 0.1520219301975148, -1.607911064798659, -0.07193516616802327, 0.3915826375219599, -1.568347410379195])
jointPositionsP2 = 1*np.array([0.644405747733462106, 1.15814689840694254, 0.1520219301975148, -1.607911064798659, -0.07193516616802327, 0.3915826375219599, -1.568347410379195]) 
jointPositionsP3 = 1*np.array([1.344405747733462106, 1.19814689840694254, 0.1520219301975148, -1.607911064798659, -0.07193516616802327, 0.3915826375219599, -1.568347410379195])
jointPositionsHome = 1*np.array([0.644405747733462106, 0.69814689840694254, 0.1520219301975148, -1.607911064798659, -0.07193516616802327, 0.7915826375219599, -1.568347410379195])

pick1 = [jointPositionsP1, jointPositionsHome]
pick2 = [jointPositionsP2, jointPositionsHome]
pick3 = [jointPositionsP3, jointPositionsHome]
place1 = [jointPositionsP1, jointPositionsHome]
place2 = [jointPositionsP2, jointPositionsHome]
place3 = [jointPositionsP3, jointPositionsHome]

class PdCtrl:
   def __init__(self):
      rospy.init_node('pd_controller')
      #self.current_joint_state = JointState()
      #self.current_ft_state = WrenchStamped()# record current state
      self.ctrl_freq = 200 #achieve the cmd in 0.1 sec
      #self.ctrl_freq = 1

      self.joint_names = ["iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"]

      #self.next_joint_state.joint_names = joint_names
      #joint_state_sub = message_filters.Subscriber('/iiwa/joint_states', numpy_msg(JointState))
      #ft_state_sub = message_filters.Subscriber('/ft_sensor/netft_data',numpy_msg(WrenchStamped))

      #self.times_sync = message_filters.ApproximateTimeSynchronizer([joint_state_sub,ft_state_sub], 1000,slop=0.003)
      #self.lin_ds_out_pub =  rospy.Publisher('/lin_ds_out', numpy_msg(DsOutput),queue_size=10)
      self.joint_cmd_pub =  rospy.Publisher('/iiwa/PositionController/command', numpy_msg(Float64MultiArray),queue_size=10) #topic important
      # self.joint_cmd_result = rospy.Publisher('/lin_ds_cmd_result', numpy_msg(JointState),queue_size=10)

   # def state_subscriber(self,joint_state,ft_state):
   #     self.current_joint_state.position = joint_state.position
   #     self.current_joint_state.velocity = joint_state.velocity
   #     self.current_ft_state = ft_state

       #self.lin_ds()
   def send_cmd(self,command,time_stamp): #command = Float64MultiArray
       next_joint_state = Float64MultiArray()

       next_joint_state.data = command
       # self.lin_ds_out.cmd = next_joint_state # pas utile 
       # self.lin_ds_out.header.stamp = time_stamp # pas utile 
       self.joint_cmd_pub.publish(next_joint_state)

   def lin_ds(self,current_joint_position, target_state):
       #target_state = 1*np.array([-0.044405747733462106, 0.49814689840694254, 0.1520219301975148, -1.007911064798659, -0.07193516616802327, 1.4915826375219599, -1.568347410379195])
       #A = 0.9 * np.eye(7,7)
       #k = 0.001
       k = 0.1
       A = np.array([[-k, 0, 0, 0, 0, 0, 0],
                     [0, -k, 0, 0, 0, 0, 0],
                     [0, 0, -k, 0, 0, 0, 0],
                     [0, 0, 0, -k, 0, 0, 0],
                     [0, 0, 0, 0, -k, 0, 0],
                     [0, 0, 0, 0, 0, -k, 0],
                     [0, 0, 0, 0, 0, 0, -k]])
       #target_state = np.array([-0.21402423525715478, 0.7634471979740207, -0.12009793253495818, -1.474234093773235, 0.09019634454611133, 0.8628468760288349, -1.8119070076103259])
       B = current_joint_position - target_state
       B = np.transpose(B)
       #next_joint_state = np.matmul(A, B) + target_state
       #speed = np.matmul(A, B)
       #next_joint_state = current_joint_position + 100*speed #100 = un pas de temps ? 
       next_joint_state = current_joint_position + np.matmul(A, B)
       #next_joint_state = np.matmul(A, B)
       return next_joint_state

   def do_action(self,action) :
        joint_states_msg = rospy.wait_for_message('iiwa/joint_states', numpy_msg(JointState))
        current_position = joint_states_msg.position
        if action == "pick(1)" :
            desired_joint_state_1 = pick1[0]
            desired_joint_state_2 = pick1[1]

        elif action == "pick(2)" :
            desired_joint_state_1 = pick2[0]
            desired_joint_state_2 = pick2[1]

        elif action == "pick(3)" :
            desired_joint_state_1 = pick3[0]
            desired_joint_state_2 = pick3[1]

        elif action == "place(1)" :
            desired_joint_state_1 = place1[0]
            desired_joint_state_2 = place1[1]

        elif action == "place(2)" :
            desired_joint_state_1 = place2[0]
            desired_joint_state_2 = place2[1]

        elif action == "place(3)" :
            desired_joint_state_1 = place3[0]
            desired_joint_state_2 = place3[1]        

        while not np.allclose(desired_joint_state_1,current_position, 1e-4, 1e-4) :
            # print(desired_joint_state)
            # print(current_position)
            next_joint_state = controller.lin_ds(current_position, desired_joint_state_1)
            controller.send_cmd(next_joint_state,time)
            r.sleep()
            joint_states_msg = rospy.wait_for_message('iiwa/joint_states', numpy_msg(JointState))
            current_position = joint_states_msg.position

        while not np.allclose(desired_joint_state_2,current_position, 1e-4, 1e-4) :
            # print(desired_joint_state)
            # print(current_position)
            next_joint_state = controller.lin_ds(current_position, desired_joint_state_2)
            controller.send_cmd(next_joint_state,time)
            r.sleep()
            joint_states_msg = rospy.wait_for_message('iiwa/joint_states', numpy_msg(JointState))
            current_position = joint_states_msg.position


if __name__ == '__main__':
    controller = PdCtrl()
    #controller.times_sync.registerCallback(controller.state_subscriber)
    rospy.sleep(0.01)
    while not rospy.is_shutdown():
        # controller.train_data_msg.header.stamp = rospy.Time.now()
        # controller.train_data_msg.joint_state = controller.current_joint_state
        # controller.train_data_msg.wrench = controller.current_ft_state.wrench

        # ds_output = controller.lin_ds(controller.train_data_msg.joint_state.position)
        #controller.train_data_msg.cmd.data = ds_output
        time = rospy.Time.now()
        r = rospy.Rate(controller.ctrl_freq)

        #print(current_position)
        #Load ML model

        #Do prediction using  controller.current_joint_state and controller.current_ft_state.wrench values

        #Add the ds output with the ml algorithm output
        # next_joint_state = 1*np.array([-0.044405747733462106, 0.49814689840694254, 0.1520219301975148, -1.007911064798659, -0.07193516616802327, 1.4915826375219599, -1.568347410379195])

        #P1
        #next_joint_state = 1*np.array([-0.044405747733462106, 1.19814689840694254, 0.1520219301975148, -1.607911064798659, -0.07193516616802327, 0.3915826375219599, -1.568347410379195]) 
        
        #P2
        #next_joint_state = 1*np.array([0.644405747733462106, 1.15814689840694254, 0.1520219301975148, -1.607911064798659, -0.07193516616802327, 0.3915826375219599, -1.568347410379195]) 

        #P3
        #next_joint_state = 1*np.array([1.344405747733462106, 1.19814689840694254, 0.1520219301975148, -1.607911064798659, -0.07193516616802327, 0.3915826375219599, -1.568347410379195]) 

        #Home
        # next_joint_state = 1*np.array([0.644405747733462106, 0.69814689840694254, 0.1520219301975148, -1.607911064798659, -0.07193516616802327, 0.7915826375219599, -1.568347410379195]) 

        # action = go to position i and then go home

        action = "pick(2)"
        controller.do_action(action)

        action = "place(1)"
        controller.do_action(action)

        action = "pick(3)"
        controller.do_action(action)

        action = "place(2)"
        controller.do_action(action)


        # while not np.allclose(desired_joint_state,current_position, 1e-4, 1e-4) :
        #     # print(desired_joint_state)
        #     # print(current_position)
        #     next_joint_state = controller.lin_ds(current_position, desired_joint_state)
        #     controller.send_cmd(next_joint_state,time)
        #     r.sleep()
        #     joint_states_msg = rospy.wait_for_message('iiwa/joint_states', numpy_msg(JointState))
        #     current_position = joint_states_msg.position

       

        # controller.send_cmd(next_joint_state,time)
        # r.sleep()
        # next_joint_state = jointPositionsP1
        # controller.send_cmd(next_joint_state,time)
        # r.sleep()
        # next_joint_state = jointPositionsHome 
        # controller.send_cmd(next_joint_state,time)
        # r.sleep()

        #rospy.loginfo(next_joint_state)
        # r = rospy.Rate(controller.ctrl_freq)

        #controller.train_data_msg.cmd_result_state = controller.current_joint_state


        #controller.train_data_pub.publish(controller.train_data_msg)