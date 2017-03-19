import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

def main():
    rospy.init_node('robbie_walker')

    listener = tf.TransformListener()


    turtle_vel = rospy.Publisher('robbie/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/shank_footprint', '/goal_1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        # cmd.angular.z = angular
        turtle_vel.publish(cmd)

        rate.sleep()

if __name__ == '__main__':
	main()