#!/usr/bin/env python

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint 
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal, JointTrajectoryControllerState
import rospy

class Robbie:
	def __init__(self):
		self.act = actionlib.SimpleActionClient('/robbie/stand_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		rospy.loginfo('Waiting for joint trajectory controller')
		self.act.wait_for_server()
		rospy.loginfo('Loaded')
		self.msg = JointTrajectory()
		self.msg.header.frame_id = "/odom"
		self.joint_names = ["shoulder_left_joint", "shoulder_right_joint", "hip_joint", "lhm_torso_joint", "stab_joint", "knee_joint"]
		self.point = JointTrajectoryPoint()
		self.positions = []

	def sm_move(self, ud):
		if ud.command == 'up':
			self.move([0, 0, -0.8, 0, 1, 1], 10)
		if ud.command == 'down':
			self.move([0, 0, 0.7, -1, 0, 0], 5)
		return 'finish'

	def cb(self, msg):
		self.positions = msg.actual.positions
		self.sub.unregister()


	def freeze_position(self, ud):
		self.sub = rospy.Subscriber("/robbie/stand_controller/state", JointTrajectoryControllerState, self.cb)
		while self.positions == []:
			rospy.loginfo("Waiting for controller state")
			rospy.sleep(0.1)
		self.move_and_wait(self.positions, 1)
		return 'finish'


	def move(self, positions, time):
		goal = FollowJointTrajectoryGoal()
		goal.trajectory.joint_names = self.joint_names
		self.point.positions = positions
		self.point.time_from_start = rospy.Duration(time)
		goal.trajectory.points.append(self.point)
		self.act.send_goal(goal)

	def move_and_wait(self, positions, time):
		goal = FollowJointTrajectoryGoal()
		goal.trajectory.joint_names = self.joint_names
		self.point.positions = positions
		self.point.time_from_start = rospy.Duration(time)
		goal.trajectory.points.append(self.point)
		self.act.send_goal_and_wait(goal)
