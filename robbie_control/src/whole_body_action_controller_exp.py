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
		self.workspace = scipy.io.loadmat('/home/moyin/dev/catkin_ws/src/gazebo_sim/robbie_control/RobotClass/dev3/100_height_waypoints')
		self.waypoints = self.workspace['waypoints']
		self.bag_name = 'sample_bag'
		self.x0 = []
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

	def extract_trajectory(self, current_state, goal):
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

	def adjust_height(self, desired_height, time = 0.5):
		self.sub = rospy.Subscriber("/robbie/whole_body_controller/state", JointTrajectoryControllerState, self.update_pos)
		while self.x0 == []:
			rospy.loginfo("Waiting for controller state")
			rospy.sleep(0.1)

		goal = FollowJointTrajectoryGoal()
		goal.trajectory.joint_names = self.joint_names
		i = 1
		for pos in self.extract_trajectory(self.x0, desired_height):
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




def main():
	rospy.init_node('stand_controller')
	rospy.loginfo("standing trajectory initiated")
	robot = Robbie()
	robot.adjust_height(desired_height = 1)


if __name__ == '__main__':
	main()
	# bag = rosbag.Bag('sample_bag_1.bag','r')
	# for topic, msg, t in bag.read_messages(topics=['/robbie/whole_body_controller/state']):
	# 	print msg
	# bag = rosbag.Bag('aerobot_tf_1.bag','r')
	# for topic, msg, t in bag.read_messages(topics=['/tf']):
	# 	print msg

