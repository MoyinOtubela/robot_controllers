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



class HeightAdjust:
	def __init__(self, robot):
		self.sm = smach.Concurrence(outcomes = ['fail','finish'],
											default_outcome = 'finish',
											input_keys = ['command','goal'],
											child_termination_cb = height_control,
											outcome_cb = height_cb
											)
		with self.sm:
			smach.Concurrence.add('height_monitor', smach_ros.MonitorState('/robbie/height/delta', Float64, height_monitor, input_keys = ['goal']))
			smach.Concurrence.add('trajectory_gen', CBState(robot.sm_move, input_keys=['command'], outcomes=['finish','fail']))



class Controller:
	def __init__(self, robot):
		self.sm = Sequence(outcomes = ['finish','fail'], connector_outcome = 'finish')
		self.sm.userdata.goal = 0.95
		self.sm.userdata.command = []
		with self.sm:
			Sequence.add('goal', GOAL())
			Sequence.add('height_adjust', HeightAdjust(robot).sm)
			Sequence.add('freeze',  CBState(robot.freeze_position, outcomes=['finish','fail']))



def main():
	rospy.init_node('MOVE_ROBBIE', anonymous = False)
	robot = Robbie()
	sm = Controller(robot)
	sis = smach_ros.IntrospectionServer('server_name',sm.sm,'/SM_ROOT')
	sis.start()
	outcome = sm.sm.execute()
	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	main()


# class HEIGHT_MONITOR(smach.State):


# Sequence.add('goal', smach_ros.MonitorState('/robbie/height/delta', Float64, height_goal, input_keys=['goal'], output_keys = ['command']))
# smach.StateMachine.add('goal', smach_ros.MonitorState('/robbie/height/delta', Float64, height_goal, input_keys=['goal'], output_keys = ['command']),
# transitions = {'valid':'height_adjust','invalid':'height_adjust','preempted':'fail'})
# smach.StateMachine.add('height_adjust', HeightAdjust().sm, 
# 	transitions = {'finish':'finish','fail':'fail'})


# def height_goal(ud, msg):
# 	rospy.loginfo('Height = %s', msg.data)
# 	if msg.data < ud.goal:
# 		ud.command = 'up'
# 	if msg.data > ud.goal:
# 		ud.command = 'down'
# 	return False
			# smach.StateMachine.add('goal', GOAL(), transitions = {'finish':'height_adjust'})
			# smach.StateMachine.add('height_adjust', HeightAdjust().sm, transitions = {'finish':'freeze'})
			# smach.StateMachine.add('freeze',  CBState(Robbie().freeze_position, outcomes=['finish','fail']), transitions = {'finish':'finish'})
		# self.sm = smach.StateMachine(outcomes = ['finish','fail'])

			# lhm = 0, knee = 1, hip = -0.5, stab = 1, should_l = 0, should_r = 0
			# lhm = -1, knee = 0, hip = 0.8, stab = 0, should_l = 0, should_r = 0
