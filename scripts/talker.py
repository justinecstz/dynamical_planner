#Talker
#Publisher associated to GetStateWorld

!/usr/bin/env python
import rospy
from std_msgs.msg import String

#Declare that the node is publishing to the chatter topic using the message type String
pub = rospy.Publisher('chatter',String)
#Tell to rospy name of the node 
rospy.init_node('talker',anonymous=True)

rate = rospy.Rate(10) #10 Hz

while not rospy.is_shutdown():
    #state = ... le string venant de GetStateWorld indiquant l'état actuel
    
    #Messages get printed to screen, get written to the Node's log file, get written to rosout (useful for debugging, use rqt_console to pulling up messages)
    rospy.loginfo(state)
    pub.publish(state)
    rate.sleep()

#???
if __name__ = '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    