#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"

#include <sstream>

// define topics that we want to publish on - ie. joint topics
ros::Publisher shank_drive_pub;	
ros::Publisher stab_pub;	
ros::Publisher knee_pub;	
ros::Publisher hip_pub;	
ros::Publisher lhm_pub;	
ros::Publisher lhm_drive_pub;	
ros::Publisher l_shoulder_pub;	
ros::Publisher r_shoulder_pub;	
ros::Publisher l_arm_pub;	
ros::Publisher r_arm_pub;	

// define message types of the topics that we want to publish on - ie. joint topics
geometry_msgs::Twist shank_drive_msg;
std_msgs::Float64 stab_msg;
std_msgs::Float64 knee_msg;
std_msgs::Float64 hip_msg;
std_msgs::Float64 lhm_msg;
std_msgs::Float64 lhm_drive_msg;
std_msgs::Float64 r_shoulder_msg;
std_msgs::Float64 l_shoulder_msg;
std_msgs::Float64 r_arm_msg;
std_msgs::Float64 l_arm_msg;

void defaultPosition();	// declare function that will set default position of joints

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_position_cpp");	// initiate 'joint_position_cpp' node
  ros::NodeHandle n;

	// define publisher objects
  shank_drive_pub = n.advertise<geometry_msgs::Twist>("/robbie/cmd_vel", 1000);	
  //lhm_drive_pub = n.advertise<std_msgs::Twist>("/robbie/lhm_cmd_vel", 1000);	// TO DO: MAKE DIFF CONTROLLER

  stab_pub = n.advertise<std_msgs::Float64>("stab_position_controller/command", 1000);	
  knee_pub = n.advertise<std_msgs::Float64>("/robbie/knee_position_controller/command", 1000);	
  hip_pub = n.advertise<std_msgs::Float64>("/robbie/hip_position_controller/command", 1000);	
  lhm_pub = n.advertise<std_msgs::Float64>("/robbie/lhm_position_controller/command", 1000);	
  l_shoulder_pub = n.advertise<std_msgs::Float64>("left_shoulder_position_controller/command", 1000);	
  r_shoulder_pub = n.advertise<std_msgs::Float64>("right_shoulder_position_controller/command", 1000);	
	l_arm_pub = n.advertise<std_msgs::Float64>("/robbie/left_arm_position_controller/command", 1000);	
  r_arm_pub = n.advertise<std_msgs::Float64>("/robbie/right_arm_position_controller/command", 1000);	

	ros::Rate loop_rate(10);	// set frequency

  while (ros::ok())
  {
    //ROS_INFO("%s", msg.data.c_str());

    defaultPosition();	// send joint commands

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

void defaultPosition()
{
	shank_drive_msg.linear.x = 0.05;
	shank_drive_msg.angular.z = 2.0;

	//shank_drive_msg.linear = '[2.0, 0.0, 0.0]';
	//shank_drive_msg.angular = '[0.0, 0.0, 1.0]';
	//lhm_drive_msg.data = 0.5;
	
	stab_msg.data = 0.5;
	knee_msg.data = 0.4;
	hip_msg.data = -0.3;
	lhm_msg.data = 0.0;
	r_shoulder_msg.data = -1.6;
	l_shoulder_msg.data = 0.0;
	r_arm_msg.data = -0.2;
	l_arm_msg.data = 0.0;

	shank_drive_pub.publish(shank_drive_msg);
	//lhm_drive_pub.publish(msg);

	stab_pub.publish(stab_msg);
	knee_pub.publish(knee_msg);
	hip_pub.publish(hip_msg);
	lhm_pub.publish(lhm_msg);
	l_shoulder_pub.publish(l_shoulder_msg);
	r_shoulder_pub.publish(r_shoulder_msg);
	l_arm_pub.publish(l_arm_msg);
	r_arm_pub.publish(r_arm_msg);
}

/*
rosrun control commands - fyi
rostopic pub -1 /robbie/lhm_position_controller/command std_msgs/Float64 "data: 0.5"
rostopic pub -1 /robbie/leftWheel_effort_controller/command std_msgs/Float64 "data: 0.5"
rostopic pub -1 /robbie/rightWheel_effort_controller/command std_msgs/Float64 "data: 0.5"
rostopic pub -1 /robbie/lhm_left_wheel_effort_controller/command std_msgs/Float64 "data: 0.5"
rostopic pub -1 /robbie/lhm_right_wheel_effort_controller/command std_msgs/Float64 "data: 0.5"

rostopic pub -1 /robbie/knee_position_controller/command std_msgs/Float64 "data: 0.5"
rostopic pub -1 /robbie/hip_position_controller/command std_msgs/Float64 "data: 0.5"
rostopic pub -1 /robbie/stab_position_controller/command std_msgs/Float64 "data: 0.5"
rostopic pub -1 /robbie/left_arm_position_controller/command std_msgs/Float64 "data: 0.5"
rostopic pub -1 /robbie/right_arm_position_controller/command std_msgs/Float64 "data: 0.5"
rostopic pub -1 /robbie/left_shoulder_position_controller/command std_msgs/Float64 "data: 0.5"
rostopic pub -1 /robbie/right_shoulder_position_controller/command std_msgs/Float64 "data: 0.5"*/

