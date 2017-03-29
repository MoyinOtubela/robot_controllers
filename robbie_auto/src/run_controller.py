#!/usr/bin/env python

import rospy
import smach
import smach_ros
from smach import Concurrence
from smach import StateMachine
from smach import Sequence
from smach import CBState
from std_msgs.msg import Float64



class dummy(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes = ['finish','fail']
			)
		self.outcome=''

	def execute(self, userdata):
		rospy.loginfo('Starting up with GOAL!')
		return 'finish'
#####################################################################################

# POSSIBLE ACTIONLIB STATE OUTCOMES
# PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST
# 

##########################		STAND 		##########################################



class DEFAULT_STAND_SM(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes = ['finish','fail']
			)

	def execute(self, userdata):
		rospy.loginfo('Moving to default mode A position')
		return 'finish'

class DEFAULT_MODE_A_STAND_SM(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes = ['finish','fail']
			)

	def execute(self, userdata):
		rospy.loginfo('Moving to default mode A position')
		return 'finish'


class HEIGHT_ADJUST_HEIGHT_ADJUST_BEHAVIOR_STAND_SM(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes = ['finish','fail']
			)

	def execute(self, userdata):
		rospy.loginfo('Moving to default mode A position')
		return 'finish'



class MODE_A_STAND_SM(smach.state_machine.StateMachine):
	def __init__(self):
		smach.state_machine.StateMachine.__init__(self, outcomes = ['finish','fail'])
		self.open()
		self.add('DEFAULT', DEFAULT_STAND_SM(), transitions = {'finish':'finish','fail':'fail'})
		self.close()


def height_control(outcome_map):
	if outcome_map['height_adjust'] == 'finish':
		return True
	return False

def height_cb(outcome_map):
	if outcome_map['height_adjust'] == 'finish':
		return 'finish'
	return 'finish'

class HEIGHT_ADJUST_BEHAVIOR_STAND_SM(smach.concurrence.Concurrence):
	def __init__(self):
		smach.concurrence.Concurrence.__init__(self, outcomes = ['finish','fail'],
			default_outcome='finish',
			child_termination_cb = height_control,
			outcome_cb = height_cb
			# outcome_map={'finish':{
			# 'height_adjust':'finish'
			# }}
			)

		self.open()
		self.add('keep_position', dummy())
		self.add('height_adjust', dummy())
		self.close()

class DELIBERATOR_STAND_SM(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes = ['mode_a','height_adjust_behavior','finish','fail']
			)
		self.plan = ['height_adjust_behavior','mode_a','finish']
		self.iter = -1

	def execute(self, userdata):
		self.iter+=1
		rospy.logwarn(self.plan)
		rospy.sleep(1)
		return self.plan[self.iter]

class STAND_SM(smach.state_machine.StateMachine):
	def __init__(self):
		smach.state_machine.StateMachine.__init__(self, outcomes = ['finish','fail'])
		self.open()
		self.add('deliberator', DELIBERATOR_STAND_SM(),transitions={'mode_a':'mode_a','height_adjust_behavior':'height_adjust_behavior','finish':'finish','fail':'fail'})
		self.add('height_adjust_behavior', HEIGHT_ADJUST_BEHAVIOR_STAND_SM(), transitions = {'finish':'deliberator','fail':'fail'})
		self.add('mode_a', MODE_A_STAND_SM(), transitions = {'finish':'deliberator','fail':'fail'})
		self.close()
##########################		STAND 		##########################################

##########################		STEP CLIMBIMG		##########################################

class DEFAULT_MODE_A_STEP_CLIMB(smach.State):
	def __init__(self, robot):
		smach.State.__init__(self,
			outcomes = ['finish','fail'],
			output_keys=['status']
			)
		self.robot = robot

	def execute(self, userdata):
		rospy.loginfo('Going into default position in MODE A')
		if self.robot.start(time=2)=='SUCCEEDED':
			userdata.status = 'SUCCEEDED'
		userdata.status = 'LOST'
		return 'finish'

class CLIMB_MODE_A_STEP_CLIMB(smach.State):
	def __init__(self, robot):
		smach.State.__init__(self,
			outcomes = ['finish','fail'],
			output_keys=['status']
			)
		self.robot = robot

	def execute(self, userdata):
		rospy.loginfo('Securing conquered step!')
		# if self.robot.adapt(robot.stage_C2, time=1, goal_='N')=='SUCCEEDED':
		# 	userdata.status = 'SUCCEEDED'
		# else:
		# 	userdata.status = 'LOST'
		# 	return 'finish'
		if self.robot.adapt(robot.stage_C3, time=1, goal_='N')=='SUCCEEDED':
			userdata.status = 'SUCCEEDED'
			rospy.loginfo('Secured conquered step!')
		else:
			userdata.status = 'LOST'
		return 'finish'

class NAVIGATE_MODE_A_STEP_CLIMB(smach.State):
	def __init__(self, robot):
		smach.State.__init__(self,
			outcomes = ['finish','fail'],
			output_keys=['status']
			)
		self.robot = robot

	def execute(self, userdata):
		rospy.loginfo('Going into default position in MODE A')
		if self.robot.locomote_shank('shank_footprint', 'goal_1')=='SUCCEEDED':
			userdata.status = 'SUCCEEDED'
		else:
			userdata.status = 'LOST'
		return 'finish'

class CLIMB_MODE_B_STEP_CLIMB(smach.State):
	def __init__(self, robot):
		smach.State.__init__(self,
			outcomes = ['finish','fail'],
			output_keys=['status']
			)
		self.robot = robot

	def execute(self, userdata):
		rospy.loginfo('Securing conquered step!')
		if self.robot.adapt(robot.stage_C2, time=1, goal_='N')=='SUCCEEDED':
			userdata.status = 'SUCCEEDED'
		else:
			userdata.status = 'LOST'
			return 'finish'
		return 'finish'

class NAVIGATE_MODE_B_STEP_CLIMB(smach.State):
	def __init__(self, robot):
		smach.State.__init__(self,
			outcomes = ['finish','fail'],
			output_keys=['status']
			)
		self.robot = robot

	def execute(self, userdata):
		rospy.loginfo('Going into default position in MODE A')
		if self.robot.locomote_lhm_shank('shank_footprint', 'goal_3')=='SUCCEEDED':
			userdata.status = 'SUCCEEDED'
		else:
			userdata.status = 'LOST'
		return 'finish'


class CLIMB_MODE_C_STEP_CLIMB(smach.State):
	def __init__(self, robot):
		smach.State.__init__(self,
			outcomes = ['finish','fail'],
			output_keys=['status']
			)
		self.robot = robot

	def execute(self, userdata):
		rospy.loginfo('Securing conquered step!')
		if self.robot.adapt(robot.stage_C1, time = 0.4, goal_ = 'N')=='SUCCEEDED':
			userdata.status = 'SUCCEEDED'
		else:
			userdata.status = 'LOST'
			return 'finish'
		return 'finish'

class NAVIGATE_MODE_C_STEP_CLIMB(smach.State):
	def __init__(self, robot):
		smach.State.__init__(self,
			outcomes = ['finish','fail'],
			output_keys=['status']
			)
		self.robot = robot

	def execute(self, userdata):
		rospy.loginfo('Going into default position in MODE A')
		if self.robot.locomote_lhm_shank('shank_footprint', 'goal_2')=='SUCCEEDED':
			userdata.status = 'SUCCEEDED'
		else:
			userdata.status = 'LOST'
		return 'finish'


class TRANSFORM_MODE_D_STEP_CLIMB(smach.State):
	def __init__(self, robot):
		smach.State.__init__(self,
			outcomes = ['finish','fail'],
			output_keys=['status']
			)
		self.robot = robot

	def execute(self, userdata):
		rospy.loginfo('Securing conquered step!')
		if self.robot.adapt(robot.stage_A1, time = 0.4, goal_ = 'N')=='SUCCEEDED':
			userdata.status = 'SUCCEEDED'
		else:
			userdata.status = 'LOST'
			return 'finish'
		if self.robot.adapt(robot.stage_A2, time = 0.4, goal_ = 'N')=='SUCCEEDED':
			userdata.status = 'SUCCEEDED'
		else:
			userdata.status = 'LOST'
			return 'finish'
		if self.robot.adapt(robot.stage_A3, time = 0.4, goal_ = 'N')=='SUCCEEDED':
			userdata.status = 'SUCCEEDED'
		else:
			userdata.status = 'LOST'
			return 'finish'
		return 'finish'


class DELIBERATOR_STEP_CLIMB(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes = ['mode_a','mode_b','mode_c','mode_d','finish','fail']
			)
		self.plan = ['mode_a','mode_d','mode_c','mode_b','mode_a','finish','fail']
		self.iter = -1

	def execute(self, userdata):
		self.iter+=1
		rospy.logwarn(self.plan[self.iter])
		rospy.sleep(1)
		return self.plan[self.iter]

class DELIBERATOR_MODE_A_STEP_CLIMB(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes = ['default', 'navigate', 'climb','finish','fail']
			)
		self.plan = ['default','navigate','finish','climb','finish']
		self.iter = -1

	def execute(self, userdata):
		self.iter+=1
		rospy.logwarn(self.plan[self.iter])
		rospy.sleep(1)
		return self.plan[self.iter]

class DELIBERATOR_MODE_B_STEP_CLIMB(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes = ['climb', 'navigate', 'finish','fail']
			)
		self.plan = ['climb','navigate','finish']
		self.iter = -1

	def execute(self, userdata):
		self.iter+=1
		rospy.logwarn(self.plan[self.iter])
		rospy.sleep(1)
		return self.plan[self.iter]

class DELIBERATOR_MODE_C_STEP_CLIMB(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes = ['climb', 'navigate', 'finish','fail']
			)
		self.plan = ['navigate','finish']
		self.iter = -1

	def execute(self, userdata):
		self.iter+=1
		rospy.logwarn(self.plan[self.iter])
		rospy.sleep(1)
		return self.plan[self.iter]



class MODE_A_STEP_CLIMB_SM(smach.state_machine.StateMachine):
	def __init__(self, robot):
		smach.state_machine.StateMachine.__init__(self, outcomes = ['finish','fail'], input_keys=['default_status', 'climb_status','navigate_status'])
		self.open()
		self.userdata.default_status = False
		self.userdata.climb_status = False
		self.userdata.navigate_status = False
		# self.add('deliberator', DELIBERATOR_MODE_A_STEP_CLIMB(), transitions={'default':'default','finish':'finish','fail':'fail','navigate':'navigate','climb':'climb'})
		# self.add('default', dummy(),transitions={'finish':'deliberator','fail':'fail'})
		# self.add('climb', dummy(), transitions={'finish':'deliberator','fail':'fail'})
		# self.add('navigate', dummy(),transitions={'finish':'deliberator','fail':'fail'})
		self.add('deliberator', DELIBERATOR_MODE_A_STEP_CLIMB(), transitions={'default':'default','finish':'finish','fail':'fail','navigate':'navigate','climb':'climb'})
		self.add('default', DEFAULT_MODE_A_STEP_CLIMB(),transitions={'finish':'deliberator','fail':'fail'})
		self.add('climb', CLIMB_MODE_A_STEP_CLIMB(), transitions={'finish':'deliberator','fail':'fail'})
		self.add('navigate', NAVIGATE_MODE_A_STEP_CLIMB(),transitions={'finish':'deliberator','fail':'fail'})
		self.close()

class MODE_B_STEP_CLIMB_SM(smach.state_machine.StateMachine):
	def __init__(self):
		smach.state_machine.StateMachine.__init__(self, outcomes = ['finish','fail'])
		self.open()
		self.add('deliberator', DELIBERATOR_MODE_B_STEP_CLIMB(), transitions={'climb':'climb','navigate':'navigate'})
		self.add('climb', dummy(),transitions={'finish':'deliberator','fail':'fail'})
		self.add('navigate', dummy(),transitions={'finish':'deliberator','fail':'fail'})
		self.close()

class MODE_C_STEP_CLIMB_SM(smach.state_machine.StateMachine):
	def __init__(self):
		smach.state_machine.StateMachine.__init__(self, outcomes = ['finish','fail'])
		self.open()
		# self.add('deliberator', DELIBERATOR_MODE_C_STEP_CLIMB(), transitions={'climb':'climb','navigate':'navigate'})
		self.add('deliberator', DELIBERATOR_MODE_C_STEP_CLIMB(), transitions={'climb':'climb','navigate':'navigate'})
		self.add('climb', dummy(), transitions={'finish':'deliberator','fail':'fail'})
		self.add('navigate', dummy(),transitions={'finish':'deliberator','fail':'fail'})
		self.close()

class MODE_D_STEP_CLIMB_SM(smach.state_machine.StateMachine):
	def __init__(self):
		smach.state_machine.StateMachine.__init__(self, outcomes = ['finish','fail'])
		self.open()
		self.add('transform', dummy(),transitions={'finish':'finish', 'fail':'fail'})
		self.close()



class STEP_CLIMB_SM(smach.state_machine.StateMachine):
	def __init__(self):
		smach.state_machine.StateMachine.__init__(self, outcomes = ['finish','fail'])
		self.open()
		self.add('deliberator', DELIBERATOR_STEP_CLIMB(),transitions={
			'mode_a':'mode_a',
			'mode_b':'mode_b',
			'mode_c':'mode_c','mode_d':'mode_d',
			'finish':'finish','fail':'fail'}
			)
		self.add('mode_a', MODE_A_STEP_CLIMB_SM(),transitions={'finish':'deliberator','fail':'fail'})
		self.add('mode_b', MODE_B_STEP_CLIMB_SM(),transitions={'finish':'deliberator','fail':'fail'})
		self.add('mode_c', MODE_C_STEP_CLIMB_SM(),transitions={'finish':'deliberator','fail':'fail'})
		self.add('mode_d', MODE_D_STEP_CLIMB_SM(),transitions={'finish':'deliberator','fail':'fail'})
		self.close()


##########################		STEP CLIMBING		##########################################


class TERRAIN_PERCEPTION(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes = ['height','step','finish','fail']
			# outcomes = ['height','finish','fail']
			)
		# self.plan = ['height','finish']
		self.plan = ['step','height','finish']
		self.iter = 0
	def execute(self, userdata):
		# rospy.loginfo('Need to change height')
		rospy.loginfo('Step detected!')
		rospy.sleep(1)
		plan = self.plan[self.iter]
		self.iter+=1
		return plan

class Controller(smach.state_machine.StateMachine):
	def __init__(self):
		smach.state_machine.StateMachine.__init__(self, outcomes = ['finish','fail'])
		self.open()
		self.add('terrain_perception', TERRAIN_PERCEPTION(), transitions = {'height':'stand_sm','step':'step_climb_sm','finish':'finish'})
		self.add('stand_sm', STAND_SM(), transitions={'finish':'terrain_perception','fail':'fail'})
		self.add('step_climb_sm', STEP_CLIMB_SM(), transitions={'finish':'terrain_perception','fail':'fail'})
		self.close()



def main():
	rospy.init_node('autonomous_controller', anonymous = False)
	# robot = Robbie()
	sm = Controller()
	sis = smach_ros.IntrospectionServer('server_name',sm,'/SM_ROOT')
	sis.start()
	outcome = sm.execute()
	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	main()
