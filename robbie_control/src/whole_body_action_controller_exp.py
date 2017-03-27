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

# from robbie_auto.msg import Script



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
		directory_height = '/home/moyin/dev/autonomous_controllers/src/robot_controllers/robbie_control/RobotClass/dev/modeA/trajectories/stand/'
		

		self.workspaceH = scipy.io.loadmat(directory_height+'/height_adjustment')

		self.height_waypoints = self.workspaceH['stage_1']

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
			# d += (current_state[i-1] - pos[i])**2
			d += (current_state[i-1] - pos[i])**2
		return d





	def extract_height_trajectory(self, current_state, goal):
		index = 0
		height_list = []
		current_state_list = []

		for pos in self.height_waypoints:
			height_list.append( CostList(index, (pos[0] - goal)**2 ) )
			current_state_list.append(CostList(index, self.minimum_distance(current_state, pos)))
			index+=1

		height_list.sort(key = CostList.getCost)
		current_state_list.sort(key = CostList.getCost)

		print height_list
		print current_state_list

		height_index = height_list[0].index
		current_state_index = current_state_list[0].index

		print [current_state_index, height_index]
		# return 0
		
		if (height_index > current_state_index):
			trajectory = self.height_waypoints[current_state_index:height_index]
			return trajectory
		trajectory = self.height_waypoints[height_index:current_state_index]
		return trajectory[::-1]



	def update_pos(self, msg):
		self.x0 = msg.actual.positions
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

		# self.bag = rosbag.Bag('sample_bag_'+str(desired_height)+'.bag','w')
		
		# tf_bag_command = 'rosbag record -O aerobot_tf_100w_'+str(desired_height)+' /tf /robbie/whole_body_controller/state'

		# tfbag = subprocess.Popen(tf_bag_command, stdin=subprocess.PIPE, shell=True,
		#  cwd='/home/moyin/dev/autonomous_controllers/src/robot_controllers/robbie_control/src')
		
		# self.sub_pose = rospy.Subscriber('/robbie/whole_body_controller/state', JointTrajectoryControllerState, self.write_pose_to_rosbag)
		# rospy.sleep(5)

		# self.sub_pose.unregister()
		# rospy.sleep(10)

		# tfbag.send_signal(subprocess.signal.SIGINT)
		# tf_reindex_command = 'rosbag reindex aerobot_tf_100w_'+str(desired_height)+'.bag.active'
		# tfbag = subprocess.Popen(tf_reindex_command, stdin=subprocess.PIPE, shell=True,
		#  cwd='/home/moyin/dev/autonomous_controllers/src/robot_controllers/robbie_control/src')
		# self.bag.close()


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

	def locomote_shank(self, wheel_frame, goal):
		done=True
		while(done and not rospy.is_shutdown()):
			try:
				(trans,rot) = self.listener.lookupTransform(wheel_frame, goal, rospy.Time(0))
				linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
				cmd = geometry_msgs.msg.Twist()
				# if linear > 0.5:
				# 	linear = 0.5
				cmd.linear.x = linear
				cmd.angular.z = 4*math.atan2(trans[1], trans[0])
				self.cmd_vel.publish(cmd)
				# angular = 4 * math.atan2(trans[1], trans[0])

				if trans[0] < 0.01:
					done = False
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue
		return 'SUCCEEDED'

	def locomote_lhm(self, wheel_frame, goal):
		done=True
		while(done and not rospy.is_shutdown()):
			try:
				(trans,rot) = self.listener.lookupTransform(wheel_frame, goal, rospy.Time(0))
				linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
				cmd = geometry_msgs.msg.Twist()
				cmd.linear.x = linear
				angular = 4*math.atan2(trans[1], trans[0])
				# if abs(angular) > 1:
				# 	angular = math.sqrt(angular)
				cmd.angular.z = angular
				self.cmd_vel_lhm.publish(cmd)
				# angular = 4 * math.atan2(trans[1], trans[0])

				if trans[0] < 0.01:
					done = False
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue
		return 'SUCCEEDED'

	def locomote_lhm_shank(self, wheel_frame, goal):
		done=False
		while(not done and not rospy.is_shutdown()):
			try:
				(trans,rot) = self.listener.lookupTransform(wheel_frame, goal, rospy.Time(0))
				linear = 0.5* math.sqrt(trans[0] ** 2 + trans[1] ** 2)
				cmd = geometry_msgs.msg.Twist()
				cmd.linear.x = linear
				angular = 4*math.atan2(trans[1], trans[0])
				# if abs(angular) > 1:
				# 	angular = math.sqrt(angular)
				cmd.angular.z = angular
				self.cmd_vel_lhm.publish(cmd)
				self.cmd_vel.publish(cmd)
				# angular = 4 * math.atan2(trans[1], trans[0])

				if trans[0] < 0.01:
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

# class Task:
# 	def __init__(self, task, start, end):
# 		self.task = task
# 		self.start = start
# 		self.end = end

# 	def getDict:
# 		return dict(Tast=self.task, Start=)


def main():
	rospy.init_node('stand_controller')
	rospy.loginfo("Trajectory initiated")
    # mode_a_msg = robbie_auto.Script()
    # mode_b_msg = robbie_auto.Script()
    # mode_c_msg = robbie_auto.Script()
    # mode_d_msg = robbie_auto.Script()

	robot = Robbie()
#
	# rospy.loginfo('%s',robot.stop())
# 
	# rospy.loginfo(robot.start(time = 2))
	# rospy.loginfo('MODE D LHM_DOWN: %s',robot.start(time = 2, positions = [0, 0, 0, -0.2, 0, 0, 0, 0])) # MODE D

	# rospy.loginfo(robot.start(time = 2))
	# robot.adjust_height(desired_height=1, time=0.2)
	# robot.adjust_height(desired_height=0, time=0.2)

	#crevice crossing
	# robot.rotate('shank_footprint','shank_goal_crevice',0)
	# rospy.loginfo('MODE A DEFAULT: %s',robot.start(time = 2)) # MODE A DEFAULT
	# rospy.loginfo('MODE A NAVIGATE: %s',robot.locomote_shank('shank_footprint', 'shank_goal_crevice')) # MODE A NAVIGATE
	# rospy.loginfo('MODE A NAVIGATE STOP: %s',robot.stop()) #MODE A NAVIGATE
	# rospy.loginfo('MODE D TRANSFORM 1: %s',robot.adapt(robot.stage_A1, time = 1, goal_ = 'N')) #MODE D 
	# rospy.loginfo('MODE D TRANSFORM 2: %s',robot.adapt(robot.stage_A2, time = 1, goal_ = 'N')) # MODE D
	# rospy.loginfo('MODE D TRANSFORM 3: %s',robot.adapt(robot.stage_A3, time = 1, goal_ = 'N')) # MODE D -> C
	# rospy.loginfo('MODE C NAVIGATE STOP: %s',robot.stop())
	# rospy.loginfo('MODE C NAVIGATE: %s',robot.locomote_lhm_shank('shank_footprint', 'crevice_goal')) # MODE C NAVIGATE 
	# rospy.loginfo('MODE C NAVIGATE STOP: %s',robot.stop())

	# rospy.loginfo('MODE B CLIMB: %s',robot.adapt(robot.stage_B1, time=0.2, goal_ = 'N')) #MODE B -> CLIMB
	# rospy.loginfo('MODE B CLIMB 2: %s',robot.adapt(robot.stage_B2, time=0.2, goal_ = 'N')) # MODE B -> CLIMB
	# rospy.loginfo('MODE B NAVIGATE: %s',robot.locomote_lhm('shank_footprint', 'crevice_goal_2')) # MODE B NAVIGATE
	# rospy.loginfo('MODE B NAVIGATE STOP: %s',robot.stop())
	# rospy.loginfo('MODE B CLIMB 3: %s',robot.adapt(robot.stage_B3, time=0.2, goal_ = 'N')) # MODE B CLIMB
	# rospy.loginfo('MODE A CLIMB: %s',robot.adapt(robot.stage_B4, time=0.3, goal_ = 'N')) #MODE A CLIMB

	# rospy.loginfo('MODE A DEFAULT: %s',robot.start(time = 1)) # MODE A CLIMB
	# robot.stop()

	# robot.adapt()


	# step climbing
	# robot.rotate('shank_footprint','goal_1', 0)
	# rospy.loginfo(robot.start(time = 2))
	# rospy.loginfo('%s', robot.locomote_shank('shank_footprint', 'goal_1'))
	# rospy.loginfo('%s',robot.stop())
	# rospy.loginfo('%s',robot.adapt(robot.stage_A1, time = 0.8, goal_ = 'N'))
	# rospy.loginfo('%s',robot.adapt(robot.stage_A2, time = 0.8, goal_ = 'N'))
	# rospy.loginfo('%s',robot.adapt(robot.stage_A3, time = 0.8, goal_ = 'N'))
	# rospy.loginfo('%s',robot.locomote_lhm_shank('shank_footprint', 'goal_2'))
	# rospy.loginfo('%s',robot.stop())
	# rospy.loginfo('%s',robot.adapt(robot.stage_C1, time = 0.8, goal_ = 'N'))
	# # rospy.loginfo('%s',robot.stop())
	# rospy.loginfo('%s',robot.adapt(robot.stage_C2, time = 0.8, goal_ = 'N'))
	# rospy.loginfo('%s',robot.locomote_lhm_shank('shank_footprint', 'goal_3'))
	# rospy.loginfo('%s',robot.adapt(robot.stage_C3, time = 0.8, goal_ = 'N'))
	# rospy.loginfo('%s',robot.stop())
	# rospy.loginfo(robot.start(time = 2))

    # Manual control  -- STEP CLIMBING 
	rospy.loginfo('MODE A DEFAULT: %s',robot.start(time = 2)) # MODE A DEFAULT
	rospy.loginfo('%s', robot.locomote_shank('shank_footprint', 'goal_1'))
	rospy.loginfo('%s', robot.stop())
	rospy.loginfo('MODE D LHM_DOWN: %s',robot.start(time = 2, positions = [0, 0, 0, -0.205, 0, 0, 0, 0])) # MODE D
	rospy.loginfo('MODE D LHM_DOWN: %s',robot.start(time = 2, positions = [0.7, 0, 0.3, -0.205, 0, 0, 0, 0])) # MODE D
	rospy.loginfo('MODE D LHM_DOWN: %s',robot.start(time = 2, positions = [0.8, 0, 0.3, -0.2, 0, 0, 0, 0])) # MODE D
	rospy.loginfo('MODE D STAB_IN TORSO BACK: %s',robot.start(time = 2, positions = [1.35, 0.2, 0.3, -0.2, 0, 0, 0, 0])) # MODE D
	rospy.loginfo('MODE C STAB_UP: %s',robot.start(time = 2, positions = [0, 0.2, 0.3, -0.2, 0, 0, 0, 0])) # MODE D
	rospy.loginfo('%s', robot.locomote_lhm_shank('shank_footprint', 'm_goal_1'))
	rospy.loginfo('%s', robot.stop())
	rospy.loginfo('MODE B STAB_DOWN: %s',robot.start(time = 2, positions = [0.9, 0, 0.3, -0.24, 0, 0, 0, 0])) # MODE D
	rospy.loginfo('MODE B STAB_DOWN: %s',robot.start(time = 2, positions = [0.5, 0, 0.3, -0.32, 0, 0, 0, 0])) # MODE D
	rospy.loginfo('%s',robot.locomote_lhm_shank('shank_footprint', 'goal_3'))
	rospy.loginfo('%s', robot.stop())
	rospy.loginfo(robot.start(time = 2))


    # Manual control  -- CREVICE CROSSING 

 #    mode_a_msg.mission[0] = 'DEFAULT'
 #    mode_a_msg.entry_status[0] = 'PENDING'
 #    mode_a_msg.begin[0] = rospy.Time.now()
 #    mode_a_msg.end[0] = rospy.Time.now()
 #    mode_a_msg.exit_status[0] = msg

    

 #    msg = robot.start(time = 2)
	# rospy.loginfo('MODE A DEFAULT: %s', robot.start(time = 2)) # MODE A DEFAULT
	# rospy.loginfo('%s', robot.locomote_shank('shank_footprint', 'shank_goal_crevice'))
	# rospy.loginfo('%s', robot.stop())

	# rospy.loginfo('MODE D LHM_DOWN: %s',robot.start(time = 2, positions = [0.8, 0, 0.3, -0.205, 0, 0, 0, 0])) # MODE D
	# rospy.loginfo('MODE D LHM_DOWN: %s',robot.start(time = 2, positions = [0.8, 0, 0.3, -0.2, 0, 0, 0, 0])) # MODE D
	# rospy.loginfo('MODE D STAB_IN TORSO BACK: %s',robot.start(time = 2, positions = [1.35, 0.2, 0.3, -0.2, 0, 0, 0, 0])) # MODE D
	# rospy.loginfo('MODE C STAB_UP: %s',robot.start(time = 2, positions = [0.4, 0.2, 0.3, -0.2, 0, 0, 0, 0])) # MODE D
	# rospy.loginfo('%s', robot.locomote_lhm_shank('shank_footprint', 'crevice_goal'))
	# rospy.loginfo('%s', robot.stop())
	# # rospy.loginfo('MODE B STAB_DOWN: %s',robot.start(time = 5, positions = [0, 0, 0.3, -0.24, 0, 0, 0, 0])) # MODE D
	# rospy.loginfo('MODE B STAB_DOWN: %s',robot.start(time = 5, positions = [0.4, 0, 0, -0.23, 0, 0, 0, 0])) # MODE D
	# rospy.loginfo('%s',robot.locomote_lhm('shank_footprint', 'm_crevice_goal_2'))
	# rospy.loginfo('%s',robot.locomote_lhm_shank('shank_footprint', 'goal_3'))
	rospy.loginfo('%s', robot.stop())
	# rospy.loginfo('MODE B STAB_DOWN: %s',robot.start(time = 5, positions = [0, 0.1, 0, -0.03, 0, 0, 0, 0])) # MODE D
	rospy.loginfo('MODE B STAB_DOWN: %s',robot.start(time = 5, positions = [0, 0.1, 0, -0.23, 0, 0, 0, 0])) # MODE D
	rospy.loginfo('MODE B STAB_DOWN: %s',robot.start(time = 2)) # MODE D
	rospy.loginfo(robot.start(time = 2))

	# rospy.loginfo('MODE C STAB_UP: %s',robot.start(time = 2, positions = [1.2, 0.15, 0, -0.24, 0, 0, 0, 0])) # MODE D
	# rospy.loginfo('MODE C STAB UP: %s',robot.start(time = 2), x0 = [0, 0, 0, -0.2, 0, 0, 0, 0]) # MODE D

	## robot.rotate('shank_footprint','goal_3',0)

	


if __name__ == '__main__':
	main()
	# bag = rosbag.Bag('sample_bag_1.bag','r')
	# for topic, msg, t in bag.read_messages(topics=['/robbie/whole_body_controller/state']):
	# 	print msg
	# bag = rosbag.Bag('aerobot_tf_1.bag','r')
	# for topic, msg, t in bag.read_messages(topics=['/tf']):
	# 	print msg


	# x0 = [0, 0, 0, -0.15, 0, 0, 0, 0]
	# x0 = [0.8, 0, 0.4047, -0.2, 0, 0, 0, 0] #x0 = [0.8, 0, 0, -0.2, 0, 0, 0, 0]
	# x0 = [0.8, 0, 0.3, -0.2, 0, 0, 0, 0]
	# x0 = [1.2, 0, 0.5, -0.2, 0, 0, 0, 0]
	# x0 = [1.5, 0.1, 0.3, -0.2, 0, 0, 0, 0]
	# x0 = [1.5, 0.2, 0.3, -0.2, 0, 0, 0, 0]
	# x0 = [0, 0.2, 0.5, -0.2, 0, 0, 0, 0]
	# x0 = [0.7, 0.2, 0.5, -0.2, 0, 0, 0, 0]

	# x0 = [0.8, 0, 0.1798, -0.2, -1.8726, -1.8726, 0, 0]
	# x0 = [1.2, 0, 0.1798, -0.2, -1.8726, -1.8726, 0, 0]
	# x0 = [1.2, 0.2, 0.1798, -0.2, -1.8726, -1.8726, 0, 0]
	# x0 = [0.8, 0, 0.3, -0.2, 0, 0, 0, 0]
	# x0 = [0.8, 0, 0.3823, 0, -1.8360, -1.8360, 0, 0]
	# x0 = [1.1937, 0, 0.5175, -0.15, -0.8724, -0.8724, 0, 0]
	# x0 = [1.1937, 0.1, 0.1121, -0.15, -0.7924, -0.7924, 0, 0] #	x0 = [1.1937, 0.0144, 0.1121, -0.2, -0.7924, -0.7924, 0, 0]

	# x0 = [1.2, 0.1, 0.25, -0.15, -0.7924, -0.7924, 0, 0]
	# x0 = [1.2, 0.1, 0.25, -0.2, -0.7924, -0.7924, 0, 0]
	# x0 = [1.2, -0.4, 0.25, -0.2, -0.7924, -0.7924, 0, 0]
	# x0 = [1.5, -0.4, 0.25, -0.2, -0.7924, -0.7924, 0, 0]
	# x0 = [1.5, -0.8, 0.7, -0.2, -0.7924, -0.7924, 0, 0]
	# x0 = [1.5, 0.1, 0.7, -0.3, -0.7924, -0.7924, 0, 0]
	# x0 = [1, 0.1, 0.7, -0.3, -0.7924, -0.7924, 0, 0]
	# robot.start(time = 2)
	# x0 = [1.2, 0.2, 0.1798, -0.2, -1.8726, -1.8726, 0, 0]
	# x0 = [1.2, 0.2, 0.1798, -0.4, -1.8726, -1.8726, 0, 0]

	# robot.start(x0, time = 2)
# # 

	# 
	# robot.adjust_height(desired_height = 1)
	# robot.adjust_height(desired_height = 1, time = 0.1)
