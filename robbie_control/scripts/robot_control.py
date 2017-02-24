#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

shank_drive_msg = Twist()
stab_msg = Float64()
knee_msg = Float64()
hip_msg = Float64()
lhm_msg = Float64()
r_shoulder_msg = Float64()
l_shoulder_msg = Float64()
r_arm_msg = Float64()
l_arm_msg = Float64()

def defaultPos():
	
	shank_drive_msg.linear.x = 0 #0.1
	shank_drive_msg.angular.z = 0  #1.5
 	
	stab_msg.data = 0.5
	knee_msg.data = 0.4
	hip_msg.data = -0.3
	lhm_msg.data = 0.0
	r_shoulder_msg.data = -1.6
	l_shoulder_msg.data = 0.0
	r_arm_msg.data = -0.2
	l_arm_msg.data = -0.2

	shank_drive_pub  = rospy.Publisher("robbie/cmd_vel", Twist, queue_size=10)
	#lhm_drive_pub  = rospy.Publisher("robbie/lhm_cmd_vel", Float64, queue_size=10)

	stab_pub = rospy.Publisher("stab_position_controller/command", Float64, queue_size=10)
	knee_pub = rospy.Publisher("/robbie/knee_position_controller/command", Float64, queue_size=10)
	hip_pub = rospy.Publisher("/robbie/hip_position_controller/command", Float64, queue_size=10)
	lhm_pub = rospy.Publisher("/robbie/lhm_position_controller/command", Float64, queue_size=10)
	l_shoulder_pub = rospy.Publisher("left_shoulder_position_controller/command", Float64, queue_size=10)
	r_shoulder_pub = rospy.Publisher("right_shoulder_position_controller/command", Float64, queue_size=10)
	l_arm_pub = rospy.Publisher("/robbie/left_arm_position_controller/command", Float64, queue_size=10)
	r_arm_pub = rospy.Publisher("/robbie/right_arm_position_controller/command", Float64, queue_size=10)

	rospy.init_node('default_position', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		shank_drive_pub.publish(shank_drive_msg)

		stab_pub.publish(stab_msg)
		knee_pub.publish(knee_msg)
		hip_pub.publish(hip_msg)
		lhm_pub.publish(lhm_msg)
		l_shoulder_pub.publish(l_shoulder_msg)
		r_shoulder_pub.publish(r_shoulder_msg)
		l_arm_pub.publish(l_arm_msg)
		r_arm_pub.publish(r_arm_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        defaultPos()
    except rospy.ROSInterruptException:
        pass

