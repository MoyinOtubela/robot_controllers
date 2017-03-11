#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import tf
import math

# subscribe to tf
# Publish distance and ||pi/2 - thetaG||


class locationPublisher:
	def __init__(self):
		self.listener = tf.TransformListener()
		self.d_pub = rospy.Publisher("/robbie/distance", Float64, queue_size = 1)
		self.o_pub = rospy.Publisher("/robbie/orientation", Float64, queue_size = 1)
		self.o_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal)
		self.pose = []

	def goal(self, msg):
		self.pose = msg.pose.position

	def run(self):

		rate = rospy.Rate(100)

		pub = rospy.Publisher('/setpoint', Float64, queue_size=1, latch = True)

		pub.publish(Float64(0))

		distance = Float64()
		orientation = Float64()

		while self.pose == []:
			rospy.loginfo('waiting for goal')
			rospy.sleep(0.5)

		four_pi = 4*math.pi

		while not rospy.is_shutdown():
			try:
				(trans, q) = self.listener.lookupTransform("odom","shank_footprint", rospy.Time(0))

				delta_x = (trans[0] - self.pose.x)
				delta_y = (trans[1] - self.pose.y)

				robot_angle = math.acos(1 - 2*q[2]**2 - 2*q[3]**2)

				distance.data = math.sqrt(delta_x**2 + delta_y**2)


				thetaG = math.atan2(delta_y, delta_x) 

				thetaR = math.pi/2 - thetaG + robot_angle

				y = ( math.cos(thetaR)*distance.data )


				orientation.data = abs(delta_y - y)
				# orientation.data = 4*math.atan2(trans)

				self.d_pub.publish(distance)
				self.o_pub.publish(orientation)

				rospy.loginfo('distance %g', distance.data)
				rospy.loginfo('orientation error %g',orientation.data)
				rospy.loginfo(q)

				rate.sleep()

			except(tf.LookupException):
				continue




def main():
	rospy.init_node("location_publisher")

	controller = locationPublisher()

	controller.run()




		


	


if __name__ == "__main__":
	main()