#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int32

def callback (data):
	rospy.loginfo (data)
	 

def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("/robbie/hip_position_controller/command", Float64, callback)
	rospy.loginfo("Test start spinning!")
	rospy.spin()

if __name__ == '__main__':
	listener()		