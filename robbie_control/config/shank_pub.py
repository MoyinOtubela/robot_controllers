#!/usr/bin/env python

import rospy
import threading
from std_msgs.msg import Float64
topicVar = 1

def callback (data):
    global topicVar
    rospy.loginfo (data)
    print data
    topicVar = data.data

def listener():
    global topicVar
    rospy.Subscriber("/robbie/hip_position_controller/command", Float64, callback)
    rospy.loginfo("Test start spinning!")
    rospy.spin()    

def talker():
    global topicVar
    pub = rospy.Publisher('/robbie/knee_position_controller/command', Float64, queue_size=10)    
    rate = rospy.Rate(10) # 10hz    
    while not rospy.is_shutdown():        
        pub.publish(topicVar/2)           
        rate.sleep()

if __name__ == '__main__': 
    try:
        rospy.init_node('mike', anonymous=True)
        
        t1=threading.Thread(name ='listener', target=listener)
        t2=threading.Thread(name ='talker', target=talker)        
        t1.start()
        t2.start()        
    except rospy.ROSInterruptException:
        pass