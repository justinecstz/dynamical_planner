#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rospy.numpy_msg import numpy_msg
# from pd_ds.msg import DsOutput, TrainningData
from geometry_msgs.msg import WrenchStamped
import message_filters

import numpy as np

class PdCtrl:
   def __init__(self):
      rospy.init_node('pd_controller')
      self.current_joint_state = JointState()
      self.current_ft_state = WrenchStamped()# record current state
      # self.train_data_msg = TrainningData()
      # self.lin_ds_out = DsOutput()
      self.ctrl_freq = 200 #achieve the cmd in 0.1 sec

      self.joint_names = ["iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"]




      #self.next_joint_state.joint_names = joint_names
      self.train_data_pub = rospy.Publisher('/train_data', numpy_msg(TrainningData),queue_size=10)
      joint_state_sub = message_filters.Subscriber('/iiwa/joint_states', numpy_msg(JointState))
      ft_state_sub = message_filters.Subscriber('/ft_sensor/netft_data',numpy_msg(WrenchStamped))

      self.times_sync = message_filters.ApproximateTimeSynchronizer([joint_state_sub,ft_state_sub], 1000,slop=0.003)
      #self.lin_ds_out_pub =  rospy.Publisher('/lin_ds_out', numpy_msg(DsOutput),queue_size=10)
      self.joint_cmd_pub =  rospy.Publisher('/iiwa/PositionController/command', numpy_msg(Float64MultiArray),queue_size=10) #topic important
      # self.joint_cmd_result = rospy.Publisher('/lin_ds_cmd_result', numpy_msg(JointState),queue_size=10)

   def state_subscriber(self,joint_state,ft_state):
       self.current_joint_state.position = joint_state.position
       self.current_joint_state.velocity = joint_state.velocity
       self.current_ft_state = ft_state

       #self.lin_ds()
   def send_cmd(self,command,time_stamp): #command = Float64MultiArray
       next_joint_state = Float64MultiArray()

       next_joint_state.data = command
       # self.lin_ds_out.cmd = next_joint_state # pas utile 
       # self.lin_ds_out.header.stamp = time_stamp # pas utile 
       self.joint_cmd_pub.publish(next_joint_state)

   def lin_ds(self,current_joint_position):
       target_state = 1*np.array([-0.044405747733462106, 0.49814689840694254, 0.1520219301975148, -1.007911064798659, -0.07193516616802327, 1.4915826375219599, -1.568347410379195])
       #A = 0.9 * np.eye(7,7)
       A = np.array([[.92, 0, 0, 0, 0, 0, 0],
                    [0, 0.92, 0, 0, 0, 0, 0],
                    [0, 0, 0.92, 0, 0, 0, 0],
                    [0, 0, 0, 0.92, 0, 0, 0],
                    [0, 0, 0, 0, 0.92, 0, 0],
                    [0, 0, 0, 0, 0, 0.92, 0],
                    [0, 0, 0, 0, 0, 0, 0.92]])
       #target_state = np.array([-0.21402423525715478, 0.7634471979740207, -0.12009793253495818, -1.474234093773235, 0.09019634454611133, 0.8628468760288349, -1.8119070076103259])
       B = current_joint_position -target_state
       B = np.transpose(B)
       next_joint_state = np.matmul(A, B) + target_state
       return next_joint_state




   # def lqr(self):
   #     #target_state = 0.9 * np.array([1, 1, 1, 1, 1, 1, 1])
   #     #target_state = 1*np.array([-0.044405747733462106, 0.49814689840694254, 0.1520219301975148, -1.007911064798659, -0.07193516616802327, 1.4915826375219599, -1.568347410379195])
   #     target_state = np.array(
   #         [-0.21402423525715478, 0.7634471979740207, -0.12009793253495818, -1.474234093773235, 0.09019634454611133,
   #          0.8628468760288349, -1.8119070076103259])
   #     ### Implement controler control and provide next position as a JointTrajectoryPoint####
   #     A = np.array([[0.95, 0, 0, 0, 0, 0, 0],
   #                   [0, 0.95, 0, 0, 0, 0, 0],
   #                   [0, 0, 0.95, 0, 0, 0, 0],
   #                   [0, 0, 0, 0.95, 0, 0, 0],
   #                   [0, 0, 0, 0, 0.95, 0, 0],
   #                   [0, 0, 0, 0, 0, 0.95, 0],
   #                   [0, 0, 0, 0, 0, 0, 0.95]])
   #     B = 9.95 * 10 ** (-4) * np.eye(7)
   #     K = 0.4142 * np.eye(7)
   #     next_traj_point = Float64MultiArray()
   #     T = self.current_joint_state.position - target_state
   #     T = np.transpose(T)
   #     # next_traj_point.positions = [0,0,0,0,0,0]
   #     # positions at next step
   #     R = np.matmul(-B, K)
   #     next_traj_point.data = np.matmul(A - R, T) + target_state


   #     self.joint_cmd_pub.publish(next_traj_point)  # send to robot for execution



if __name__ == '__main__':
    controller = PdCtrl()
    controller.times_sync.registerCallback(controller.state_subscriber)
    rospy.sleep(0.01)

    while not rospy.is_shutdown():
        controller.train_data_msg.header.stamp = rospy.Time.now()
        controller.train_data_msg.joint_state = controller.current_joint_state
        controller.train_data_msg.wrench = controller.current_ft_state.wrench

        ds_output = controller.lin_ds(controller.train_data_msg.joint_state.position)
        controller.train_data_msg.cmd.data = ds_output
        time = rospy.Time.now()

        #Load ML model

        #Do prediction using  controller.current_joint_state and controller.current_ft_state.wrench values

        #Add the ds output with the ml algorithm output
        next_joint_state = ds_output#+ml_output

        controller.send_cmd(next_joint_state,time)

        r = rospy.Rate(controller.ctrl_freq)

        r.sleep()

        controller.train_data_msg.cmd_result_state = controller.current_joint_state


        controller.train_data_pub.publish(controller.train_data_msg)