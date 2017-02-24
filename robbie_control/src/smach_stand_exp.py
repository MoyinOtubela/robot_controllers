#!/usr/bin/env python

import rospy
import smach
import smach_ros
from smach import Concurrence
from smach import Sequence
from smach import CBState
from std_msgs.msg import Float64
from Robbie import Robbie
#biggest lesson use both input and output keys with the same names to pass data between state machines

class GOAL(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes = ['finish','fail'],
			input_keys = ['goal'],
			output_keys = ['command']
			)
		self.outcome=''

	def cb(self, msg, userdata):
		if msg.data < userdata.goal:
			userdata.command = 'up'
			rospy.loginfo("command = up")
		if msg.data > userdata.goal:
			userdata.command = 'down'
			rospy.loginfo("command = down")
		self.outcome = 'finish'
		self.sub.unregister()

	def execute(self, userdata):
		rospy.loginfo('Starting up with GOAL!')
		self.sub = rospy.Subscriber('/robbie/height/delta', Float64, self.cb, callback_args=userdata)
		while self.outcome=='':
			rospy.loginfo('Waiting for outcome decision')
			rospy.sleep(1)
		return self.outcome



def height_monitor(ud, msg):
	# rospy.loginfo('Height = %s', msg.data)
	if ((msg.data - ud.goal) >= -0.001) and ((msg.data - ud.goal) <= 0.001):
		return False
	return True

def height_control(outcome_map):
	if outcome_map['height_monitor'] == 'invalid':
		return True
	return False

def height_cb(outcome_map):
	if outcome_map['height_monitor'] == 'invalid':
		return 'finish'



class HeightAdjust(smach.concurrence.Concurrence):
	def __init__(self, robot):
		smach.concurrence.Concurrence.__init__(self, outcomes = ['fail','finish'],
											default_outcome = 'finish',
											input_keys = ['command','goal'],
											child_termination_cb = height_control,
											outcome_cb = height_cb
											)
		self.open()
		self.add('height_monitor', smach_ros.MonitorState('/robbie/height/delta', Float64, height_monitor, input_keys = ['goal']))
		self.add('trajectory_gen', CBState(robot.sm_move, input_keys=['command'], outcomes=['finish','fail']))
		self.close()


class Controller(smach.sequence.Sequence):
	def __init__(self, robot):
		smach.sequence.Sequence.__init__(self, outcomes = ['finish','fail'], connector_outcome = 'finish')
		self.open()
		self.userdata.goal = 0.95
		self.userdata.command = []
		self.add('goal', GOAL())
		self.add('height_adjust', HeightAdjust(robot))
		self.add('freeze',  CBState(robot.freeze_position, outcomes=['finish','fail']))
		self.close()

def main():
	rospy.init_node('MOVE_ROBBIE', anonymous = False)
	robot = Robbie()
	sm = Controller(robot)
	sis = smach_ros.IntrospectionServer('server_name',sm,'/SM_ROOT')
	sis.start()
	outcome = sm.execute()
	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	main()
