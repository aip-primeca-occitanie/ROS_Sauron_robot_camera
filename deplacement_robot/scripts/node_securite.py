#!/usr/bin/env python

import rospy
from industrial_msgs.msg import RobotStatus
from std_msgs.msg import String


class Secu:
    def __init__(self):
        self.status = "NOK"
        self.pub_secu = rospy.Publisher("/securite_state", String, queue_size=10)
        rospy.init_node('move_robot_server', anonymous=True)
        rospy.Subscriber("/robot_status", RobotStatus, self.callback_robot_status)

    def callback_robot_status(self,msg):
        if msg.e_stopped.val != 0 or msg.in_error.val != 0:
            self.status = "NOK"
        else :
            self.status = "OK"
	    
    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.pub_secu.publish(self.status)
            rate.sleep()


s = Secu()
s.run()