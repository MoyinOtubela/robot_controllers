#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class moveRobbie:
	def __init__(self):
		self.a_sub = rospy.Subscriber("/robbie/angular", Float64, self.ang_callback)
		self.l_sub = rospy.Subscriber("/robbie/linear", Float64, self.lin_callback)
		self.o_pub = rospy.Publisher("/robbie/cmd_vel", Twist, queue_size = 1)
		self.a = Twist()
		self.l = Twist()

		self.a.angular.x = 0
		self.a.angular.y = 0
		self.a.angular.z = 0

		self.a.linear.x = 0
		self.a.linear.y = 0
		self.a.linear.z = 0

		self.l.angular.x = 0
		self.l.angular.y = 0
		self.l.angular.z = 0

		self.l.linear.x = 0
		self.l.linear.y = 0
		self.l.linear.z = 0

	def ang_callback(self, msg):
		self.a.angular.z = msg.data
		# self.a.header = 'odom'
		self.o_pub.publish(self.a)
		rospy.loginfo(self.a)

	def lin_callback(self, msg):
		self.l.linear.x = msg.data
		# self.a.header = 'odom'
		self.o_pub.publish(self.l)
		rospy.loginfo(self.l)






def main():
	rospy.init_node('location_controller')
	moveRobbie()

	rospy.spin()


if __name__ == '__main__':
	main()