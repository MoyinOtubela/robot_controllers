#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

class Echo(object):
    def __init__(self):
        self.value = 0

        rospy.init_node('echoer')

        self.pub = rospy.Publisher('/robbie/hip_position_controller/command', Float64, latch=True)
        rospy.Subscriber('/robbie/knee_position_controller/command', Float64, self.update_value)

    def update_value(self, msg):
        self.value = msg.data

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub.publish(self.value)
            rospy.loginfo(salf.value)
            r.sleep()