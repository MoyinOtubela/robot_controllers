import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint 
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
import scipy.io
from math import sqrt

class Robbie:
	def __init__(self):
		self.act = actionlib.SimpleActionClient('/robbie/whole_body_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		rospy.loginfo('Waiting for joint trajectory controller')
		self.act.wait_for_server()
		rospy.loginfo('Loaded')
		self.msg = JointTrajectory()
		self.msg.header.frame_id = "/odom"
		# self.joint_names = ["lhm_torso_joint", "knee_joint", "hip_joint", "stab_joint", "shoulder_left_joint", "shoulder_right_joint"]
		self.joint_names = [ "stab_joint", "knee_joint", "hip_joint", "lhm_torso_joint", "shoulder_left_joint", "shoulder_right_joint", "elbow_left_joint", "elbow_right_joint"]
		self.workspace = scipy.io.loadmat('/home/moyin/dev/catkin_ws/src/gazebo_sim/robbie_control/RobotClass/dev3/100_height_waypoints')
		self.waypoints = self.workspace['waypoints']
		

	def stand(self, time):
		goal = FollowJointTrajectoryGoal()
		goal.trajectory.joint_names = self.joint_names
		i = 1
		# use if loop to reverse or go normal
		# make sure to extract all data then place in condition to see when to stop waypoints
		for pos in self.waypoints:
			goal.trajectory.points.append(JointTrajectoryPoint(
			 positions = pos[1:9],
			 time_from_start =  rospy.Duration(i*time)))
			i = i + 1

		goal.trajectory.header.stamp = rospy.Time.now()
		self.act.send_goal_and_wait(goal)

	def sit(self, time):
		goal = FollowJointTrajectoryGoal()
		goal.trajectory.joint_names = self.joint_names
		i = 1
		for pos in reversed(self.waypoints):
			goal.trajectory.points.append(JointTrajectoryPoint(
			 positions = pos[1:9],
			 time_from_start =  rospy.Duration(i*time)))
			i = i + 1

		goal.trajectory.points.append(JointTrajectoryPoint(
		 positions = [0,0,0.8,-0.2,0,0,0,0],
		 time_from_start =  rospy.Duration(i*time)))
		i = i + 1
		 # velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],

		goal.trajectory.header.stamp = rospy.Time.now()
		self.act.send_goal_and_wait(goal)


def main():
	rospy.init_node('stand_controller')
	rospy.loginfo("standing trajectory initiated")
	robot = Robbie()

	x0 = [0,0,0.8,-0.2,0,0,0,0]

	# robot.stand(5)
	# robot.sit(5)
	# robot.stand(2.5)
	# robot.sit(2.5)

	robot.stand(0.5)
	robot.sit(0.5)
	# while not rospy.is_shutdown():
	# robot.stand(0.5)
	# robot.sit(0.5)



if __name__ == '__main__':
	main()


		# for pos in self.waypoints:
		# 	positions = [pos[4], pos[2], pos[3], pos[1], pos[5], pos[6]]
		# 	velocities = self.derive(positions, time)
		# 	goal.trajectory.points.append(JointTrajectoryPoint(
		# 	 positions = positions,
		# 	 velocities = velocities,
		# 	 # accelerations = self.derive(velocities, time),
		# 	 time_from_start =  rospy.Duration(i*time)))
		# 	i = i + 1
