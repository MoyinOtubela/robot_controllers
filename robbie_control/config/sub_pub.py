#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int32
global topicVar

def callback (data):
    rospy.loginfo (data)
    topicVar = data
     

def listener():
    topicVar = 0
    rospy.init_node('shank_function', anonymous=True)
    rospy.Subscriber("/robbie/hip_position_controller/command", Float64, callback)
    rospy.loginfo("Test start spinning!")
    print topicVar
    rospy.spin()

if __name__ == '__main__':
    listener()      