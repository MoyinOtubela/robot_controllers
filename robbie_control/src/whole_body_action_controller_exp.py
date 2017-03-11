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
		directory = '/home/moyin/dev/autonomous_controllers/src/robot_controllers/robbie_control/RobotClass/dev/modeA/trajectories/modeC/'
		self.workspaceT = scipy.io.loadmat(directory+'/lbel_waypoints')
		self.waypointsT = self.workspaceT['waypoints']
		# self.workspace = scipy.io.loadmat('/home/moyin/dev/autonomous_controllers/src/robot_controllers/robbie_control/RobotClass/dev/100_height_waypoints_2')
		# self.workspace = scipy.io.loadmat('/home/moyin/dev/autonomous_controllers/src/robot_controllers/robbie_control/RobotClass/dev/100_height_waypoints_2')

		self.bag_name = 'sample_bag'
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

		print [current_state_index, height_index]
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
		print self.x0
		self.sub.unregister()

	def update_pos_T(self, msg):
		self.x0T = msg.actual.positions
		print self.x0T
		self.subT.unregister()

	def start(self, positions = [0, 0, 0, 0, 0, 0, 0, 0], time = 5):
		goal = FollowJointTrajectoryGoal()
		goal.trajectory.joint_names = self.joint_names
		goal.trajectory.points.append(JointTrajectoryPoint(
		 positions = positions,
		 time_from_start = rospy.Duration(time)))
		goal.trajectory.header.stamp = rospy.Time.now()
		self.act.send_goal_and_wait(goal)


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
		
		self.bag = rosbag.Bag('sample_bag_'+str(desired_height)+'.bag','w')
		
		tf_bag_command = 'rosbag record -O aerobot_tf_100w_'+str(desired_height)+' /tf /robbie/whole_body_controller/state'

		tfbag = subprocess.Popen(tf_bag_command, stdin=subprocess.PIPE, shell=True,
		 cwd='/home/moyin/dev/autonomous_controllers/src/robot_controllers/robbie_control/src')
		
		self.sub_pose = rospy.Subscriber('/robbie/whole_body_controller/state', JointTrajectoryControllerState, self.write_pose_to_rosbag)
		# rospy.sleep(5)

		self.act.send_goal_and_wait(goal)
		self.sub_pose.unregister()
		rospy.sleep(10)

		tfbag.send_signal(subprocess.signal.SIGINT)
		tf_reindex_command = 'rosbag reindex aerobot_tf_100w_'+str(desired_height)+'.bag.active'
		tfbag = subprocess.Popen(tf_reindex_command, stdin=subprocess.PIPE, shell=True,
		 cwd='/home/moyin/dev/autonomous_controllers/src/robot_controllers/robbie_control/src')
		self.bag.close()


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

		self.act.send_goal_and_wait(goal)

	def extract_transition_trajectory(self, trajectory_, current_state, goal):

		trajectory = []
		index = 1
		for pos in trajectory_:
			trajectory.append(CostList(pos[0], self.minimum_distance(current_state, pos)) )

		trajectory.sort(key = CostList.getIndex)

		current_state = trajectory[0].index

		print [current_state]

		if goal == 'N':
			trajectory = trajectory_[current_state::]
			return trajectory

		trajectory = trajectory_[::-1]
		return trajectory

def main():
	rospy.init_node('stand_controller')
	rospy.loginfo("standing trajectory initiated")
	robot = Robbie()
# 
	

	stage_1 = robot.waypointsT[0][0][0]
	stage_2 = robot.waypointsT[0][0][1]
	stage_3 = robot.waypointsT[0][0][2]

	# print stage_1
	# print "=============================================="
	# print "=============================================="
	# print "=============================================="
	# print "=============================================="
	# print stage_2

	# x0 = [1.1937, 0.1, 0.25, -0.15, -0.7924, -0.7924, 0, 0] #	x0 = [1.1937, 0.0144, 0.1121, -0.2, -0.7924, -0.7924, 0, 0]
	x0 = [0.5, 0.1, 0.25, -0.15, -0.7924, -0.7924, 0, 0] #	x0 = [1.1937, 0.0144, 0.1121, -0.2, -0.7924, -0.7924, 0, 0]
	# 

	# x0 = [1.2, 0.2, 0.1798, -0.2, -1.8726, -1.8726, 0, 0]

	# robot.start(time = 2)
	# rospy.sleep(1)
	# robot.adapt(stage_1, time = 0.1, goal_ = 'N')
	# rospy.sleep(1)
	# robot.adapt(stage_2, time = 0.3, goal_ = 'N')
	# rospy.sleep(1)
	# robot.adapt(stage_3, time = 1, goal_ = 'N')
	# x0 = [1.2, 0.1, 0.25, -0.15, -0.7924, -0.7924, 0, 0]
	x0 = [1.2, 0.1, 0.25, -0.4, -0.7924, -0.7924, 0, 0]
	robot.start(x0, time = 2)
# 

	# 
	# robot.adjust_height(desired_height = 1)
	# robot.adjust_height(desired_height = 1, time = 0.1)


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
