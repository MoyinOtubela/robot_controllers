#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint 
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal, JointTrajectoryControllerState
# from tf2_msgs.msg import TFMessage
from tf.msg import tfMessage
from geometry_msgs.msg import TransformStamped
import scipy.io
from math import sqrt
import rosbag
import subprocess
import math
import tf
import geometry_msgs.msg
import numpy
import thread


class CostList(object):
	def __init__(self, index, cost):
			self.index = index
			self.cost = cost
	def __repr__(self):
		return '{}: {} {}'.format(self.__class__.__name__, self.index, self.cost)

	def __cmp__(self, other):
		# if hasattr(other, 'cost'):
		return self.cost.__cmp__(other.cost)

	def getCost(self):
		return self.cost

	def getIndex(self):
		return self.index

class Robbie:
	def __init__(self):
		self.act = actionlib.SimpleActionClient('/robbie/whole_body_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		rospy.loginfo('Waiting for joint trajectory controller')
		self.act.wait_for_server()
		rospy.loginfo('Loaded')
		self.msg = JointTrajectory()
		self.msg.header.frame_id = "/odom"
		self.joint_names = ["stab_joint", "knee_joint", "hip_joint", "lhm_torso_joint", "shoulder_left_joint", "shoulder_right_joint", "elbow_left_joint", "elbow_right_joint"]
		directory_A = '/home/moyin/dev/autonomous_controllers/src/robot_controllers/robbie_control/RobotClass/dev/modeA/trajectories/modeC/'
		directory_C = '/home/moyin/dev/autonomous_controllers/src/robot_controllers/robbie_control/RobotClass/dev/modeC/trajectories/climb/'
		directory_B = '/home/moyin/dev/autonomous_controllers/src/robot_controllers/robbie_control/RobotClass/dev/modeC/trajectories/modeB/'
		self.workspaceA = scipy.io.loadmat(directory_A+'/lbel_waypoints')
		self.stage_A1 = self.workspaceA['stage_1']
		self.stage_A2 = self.workspaceA['stage_2']
		self.stage_A3 = self.workspaceA['stage_3']
		self.workspaceB = scipy.io.loadmat(directory_B+'/sdsu_waypoints')
		self.stage_B1 = self.workspaceB['stage_1']
		self.stage_B2 = self.workspaceB['stage_2']
		self.stage_B3 = self.workspaceB['stage_3']
		self.stage_B4 = self.workspaceB['stage_4']
		self.workspaceC = scipy.io.loadmat(directory_C+'/ldkc_waypoints')
		self.stage_C1 = self.workspaceC['stage_1']
		self.stage_C2 = self.workspaceC['stage_2']
		self.stage_C3 = self.workspaceC['stage_3']

# setup motor command publishers
		self.cmd_vel = rospy.Publisher('robbie/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
		self.cmd_vel_lhm = rospy.Publisher('robbie/cmd_vel_lhm', geometry_msgs.msg.Twist,queue_size=1)
		self.listener = tf.TransformListener()
		# self.workspace = scipy.io.loadmat('/home/moyin/dev/autonomous_controllers/src/robot_controllers/robbie_control/RobotClass/dev/100_height_waypoints_2')
		# self.workspace = scipy.io.loadmat('/home/moyin/dev/autonomous_controllers/src/robot_controllers/robbie_control/RobotClass/dev/100_height_waypoints_2')

		# self.bag_name = 'sample_bag'
		self.x0 = []
		self.x0T = []
		# self.bag = rosbag.Bag('sample_bag_1.bag','w')
		# self.tfbag = rosbag.Bag('aerobot_tf_1.bag','w')


	def __del__(self):
		self.act.cancel_all_goals()
		# pass

	def minimum_distance(self, current_state, pos):
		d = 0
		for i in range(1, 8):
			d += abs(current_state[i-1] - pos[i])
		return d





	def extract_height_trajectory(self, current_state, goal):
		index = 0
		height_list = []
		current_state_list = []

		for pos in self.waypoints:
			height_list.append( CostList(index, abs(pos[0] - goal) ) )
			current_state_list.append(CostList(index, self.minimum_distance(current_state, pos)))
			index+=1

		height_list.sort(key = CostList.getCost)
		current_state_list.sort(key = CostList.getCost)

		# print height_list
		# print current_state_list

		height_index = height_list[0].index
		current_state_index = current_state_list[0].index

		# print [current_state_index, height_index]
		# return 0
		
		if (height_index > current_state_index):
			trajectory = self.waypoints[current_state_index:height_index]
			return trajectory
		trajectory = self.waypoints[height_index:current_state_index]
		return trajectory[::-1]


	def write_pose_to_rosbag(self, msg):
		self.bag.write('/robbie/whole_body_controller/state', msg)
		rospy.Rate(100)


	def update_pos(self, msg):
		self.x0 = msg.actual.positions
		# print self.x0
		self.sub.unregister()

	def update_pos_T(self, msg):
		self.x0T = msg.actual.positions
		# print self.x0T
		self.subT.unregister()

	def start(self, positions = [0, 0, 0, 0, 0, 0, 0, 0], time = 5):
		goal = FollowJointTrajectoryGoal()
		goal.trajectory.joint_names = self.joint_names
		goal.trajectory.points.append(JointTrajectoryPoint(
		 positions = positions,
		 time_from_start = rospy.Duration(time)))
		goal.trajectory.header.stamp = rospy.Time.now()
		if self.act.send_goal_and_wait(goal) == 4:
			return 'SUCCEEDED'
		return 'LOST'


	def adjust_height(self, desired_height, time = 0.5):
		self.sub = rospy.Subscriber("/robbie/whole_body_controller/state", JointTrajectoryControllerState, self.update_pos)
		while self.x0 == []:
			rospy.loginfo("Waiting for controller state")
			rospy.sleep(0.1)

		goal = FollowJointTrajectoryGoal()
		goal.trajectory.joint_names = self.joint_names

		i = 1
		for pos in self.extract_height_trajectory(self.x0, desired_height):
			goal.trajectory.points.append(JointTrajectoryPoint(
			 positions = pos[1:9],
			 time_from_start = rospy.Duration(i*time)))
			i+=1

		goal.trajectory.header.stamp = rospy.Time.now()
		if self.act.send_goal_and_wait(goal) == 4:
			return 'SUCCEEDED'
		return 'LOST'


	def adapt(self, trajectory, goal_, time = 0.5):

		self.subT = rospy.Subscriber("/robbie/whole_body_controller/state", JointTrajectoryControllerState, self.update_pos_T)
		while self.x0T == []:
			rospy.loginfo("Waiting for controller state")
			rospy.sleep(0.1)

		goal = FollowJointTrajectoryGoal()
		goal.trajectory.joint_names = self.joint_names

		i = 1
		for pos in self.extract_transition_trajectory(trajectory, self.x0T, goal_):
			goal.trajectory.points.append(JointTrajectoryPoint(
			 positions = pos[1:9],
			 time_from_start = rospy.Duration(i*time)))
			i+=1

		goal.trajectory.header.stamp = rospy.Time.now()

		if self.act.send_goal_and_wait(goal) == 4:
			return 'SUCCEEDED'
		return 'LOST'


	def extract_transition_trajectory(self, trajectory_, current_state, goal):

		trajectory = []
		index = 1
		for pos in trajectory_:
			trajectory.append(CostList(pos[0], self.minimum_distance(current_state, pos)) )

		trajectory.sort(key = CostList.getIndex)

		current_state = trajectory[0].index

		# print [current_state]

		if goal == 'N':
			trajectory = trajectory_[current_state::]
			return trajectory
		# end = len(trajectory)
		trajectory = trajectory_[current_state::]
		return trajectory[::-1]


		# trajectory = trajectory_[::-1]

	def locomote_shank(self, wheel_frame, goal, time = 2):
		done=False
		start = rospy.Time.now()
		while(not done and not rospy.is_shutdown()):
			try:
				if (rospy.Time.now().to_sec() - start.to_sec()) > time:
					rospy.logwarn(rospy.Time.now().to_sec() - start.to_sec())
					self.stop()
					return 'LOST'

				(trans,rot) = self.listener.lookupTransform(wheel_frame, goal, rospy.Time(0))
				if(trans[0] < 0):
					linear = -0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
				else:
					linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
				cmd = geometry_msgs.msg.Twist()

				cmd.linear.x = linear
				angular = 4*math.atan2(trans[1], trans[0])

				# if abs(angular) > 3:
				# 	cmd.angular.z = angular/abs(angular)
				# else:
				cmd.angular.z = angular


				self.cmd_vel.publish(cmd)
				# angular = 4 * math.atan2(trans[1], trans[0])

				if math.sqrt(trans[0] ** 2 + trans[1] ** 2) < 0.01 and (abs(self.angle_turned(rot)) < 0.1):
					done = True
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue
		return 'SUCCEEDED'

	def locomote_shank_recovery(self, wheel_frame, goal, time = 2):
		done=False
		start = rospy.Time.now()
		while(not done and not rospy.is_shutdown()):
			try:
				if (rospy.Time.now().to_sec() - start.to_sec()) > time:
					rospy.logwarn(rospy.Time.now().to_sec() - start.to_sec())
					self.stop()
					return 'LOST'

				(trans,rot) = self.listener.lookupTransform(wheel_frame, goal, rospy.Time(0))
				if(trans[0] < 0):
					linear = -0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
				else:
					linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
				cmd = geometry_msgs.msg.Twist()

				cmd.linear.x = linear
				angular = 4*math.atan2(trans[1], trans[0])

				if abs(angular) > 1:
					cmd.angular.z = angular/abs(angular)
				else:
					cmd.angular.z = angular


				self.cmd_vel.publish(cmd)
				# angular = 4 * math.atan2(trans[1], trans[0])

				if math.sqrt(trans[0] ** 2 + trans[1] ** 2) < 0.01:
					done = True
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue
		return 'SUCCEEDED'

	def locomote_lhm(self, wheel_frame, goal, time = 1):
		done=False
		while(not done and not rospy.is_shutdown()):
			try:
				(trans,rot) = self.listener.lookupTransform(wheel_frame, goal, rospy.Time(0))
				linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
				cmd = geometry_msgs.msg.Twist()
				cmd.linear.x = linear
				angular = 4*math.atan2(trans[1], trans[0])
				if abs(angular) > 1:
					cmd.angular.z = angular/abs(angular)
				else:
					cmd.angular.z = angular

				cmd.angular.z = angular
				self.cmd_vel_lhm.publish(cmd)
				# angular = 4 * math.atan2(trans[1], trans[0])

				if math.sqrt(trans[0] ** 2 + trans[1] ** 2)  < 0.01:
					done = True
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue
		return 'SUCCEEDED'

	def locomote_lhm_shank(self, wheel_frame, goal, time = 1):
		done=False
		start = rospy.Time.now()

		while(not done and not rospy.is_shutdown()):
			try:
				if (rospy.Time.now() - start) > rospy.Duration(time):
					self.stop()
					return 'LOST'

				(trans,rot) = self.listener.lookupTransform(wheel_frame, goal, rospy.Time(0))
				linear = 0.5* math.sqrt(trans[0] ** 2 + trans[1] ** 2)
				cmd = geometry_msgs.msg.Twist()
				cmd.linear.x = linear
				angular = 4*math.atan2(trans[1], trans[0])
				if abs(angular) > 1:
					cmd.angular.z = angular/abs(angular)
				else:
					cmd.angular.z = angular

				cmd.angular.z = angular
				self.cmd_vel_lhm.publish(cmd)
				self.cmd_vel.publish(cmd)
				# angular = 4 * math.atan2(trans[1], trans[0])

				if math.sqrt(trans[0] ** 2 + trans[1] ** 2) < 0.01:
					done = True
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue
		return 'SUCCEEDED'


	def conjugate(self, a):
		return [-a[0], -a[1], -a[2], a[3]]


	def angle_turned(self, q):
		q_x = q[0]
		q_y = q[1]
		q_z = q[2]
		q_w = q[3]
		t3 = 2*(q_w*q_z + q_x*q_y)
		t4 = 1 - 2*(q_y*q_y + q_z*q_z)
		pitch = math.atan2(t3, t4)
		return pitch


	def quaternion_multiply(self, q1, q0):
		w0 = q0[3]
		x0 = q0[0]
		y0 = q0[1]
		z0 = q0[2]
		w1 = q1[3]
		x1 = q1[0]
		y1 = q1[1]
		z1 = q1[2]
		return [-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
		x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
		-x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
		x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0]


	def rotate(self, wheel_frame, goal, angle):

		if angle <= 0:
			angle +=2*math.pi
		if angle >= 2*math.pi:
			angle -=2*math.pi

		cmd = geometry_msgs.msg.Twist()
		speed = 0.5

		done = False
		r = rospy.Rate(10)
		while(not done and not rospy.is_shutdown()):
			self.cmd_vel.publish(cmd)

			try:
				(trans,rot) = self.listener.lookupTransform(wheel_frame, goal, rospy.Time(0))
				# d = sqrt(trans[1]**2 + trans[0]**2)
				# t = math.acos(trans[1]/d)
				t =  math.atan2(trans[1], trans[0]) 
				if t>0:
					cmd.angular.z = speed
				else:
					cmd.angular.z = -speed
				# t =  math.atan2(trans[1], trans[0]) 
				# print abs(self.angle_turned(rot) - angle)
				# print cmd
				# print rot
				if(abs(self.angle_turned(rot) - angle) < 0.05):
					done = True
					self.cmd_vel.publish(geometry_msgs.msg.Twist())
				r.sleep()
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				print 'oops!'
				continue
		return 'SUCCEEDED'

	def stop(self):
		cmd = geometry_msgs.msg.Twist()
		self.cmd_vel.publish(cmd)
		self.cmd_vel_lhm.publish(cmd)
		return 'SUCCEEDED'

# class Robbie:
# 	def __init__(self):
# 		self.act = actionlib.SimpleActionClient('/robbie/stand_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
# 		rospy.loginfo('Waiting for joint trajectory controller')
# 		self.act.wait_for_server()
# 		rospy.loginfo('Loaded')
# 		self.msg = JointTrajectory()
# 		self.msg.header.frame_id = "/odom"
# 		self.joint_names = ["shoulder_left_joint", "shoulder_right_joint", "hip_joint", "lhm_torso_joint", "stab_joint", "knee_joint"]
# 		self.point = JointTrajectoryPoint()
# 		self.positions = []

# 	def sm_move(self, ud):
# 		if ud.command == 'up':
# 			self.move([0, 0, -0.8, 0, 1, 1], 10)
# 		if ud.command == 'down':
# 			self.move([0, 0, 0.7, -1, 0, 0], 5)
# 		return 'finish'

# 	def cb(self, msg):
# 		self.positions = msg.actual.positions
# 		self.sub.unregister()


# 	def freeze_position(self, ud):
# 		self.sub = rospy.Subscriber("/robbie/stand_controller/state", JointTrajectoryControllerState, self.cb)
# 		while self.positions == []:
# 			rospy.loginfo("Waiting for controller state")
# 			rospy.sleep(0.1)
# 		self.move_and_wait(self.positions, 1)
# 		return 'finish'


# 	def move(self, positions, time):
# 		goal = FollowJointTrajectoryGoal()
# 		goal.trajectory.joint_names = self.joint_names
# 		self.point.positions = positions
# 		self.point.time_from_start = rospy.Duration(time)
# 		goal.trajectory.points.append(self.point)
# 		self.act.send_goal(goal)

# 	def move_and_wait(self, positions, time):
# 		goal = FollowJointTrajectoryGoal()
# 		goal.trajectory.joint_names = self.joint_names
# 		self.point.positions = positions
# 		self.point.time_from_start = rospy.Duration(time)
# 		goal.trajectory.points.append(self.point)
# 		self.act.send_goal_and_wait(goal)
