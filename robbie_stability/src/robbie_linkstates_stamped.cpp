#include <ros/ros.h>
#include <memory>
// #include <gazebo/transport/transport.hh>
#include <gazebo_msgs/LinkStates.h>
#include <robbie_stability/LinkStatesStamped.h>
#include <robbie_stability/Contact.h>
// #include <robbie_stability/Contact.h>
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>


namespace ros{

using namespace gazebo_msgs;
using namespace robbie_stability;

Contact contact_msg;
LinkStatesStamped msg;
ros::Publisher loc_contact_pub;
ros::Subscriber sub;
std::unique_ptr<Rate> rate;
int seq;

void callback(LinkStates link_msg){
	++seq;
	msg.states = link_msg;
	msg.header.seq = seq;
	msg.header.stamp = ros::Time::now();
	loc_contact_pub.publish(msg);
	// rate->sleep();
	// msg.frame_id = 
}


int main(int argc, char** argv)
{
	init(argc, argv,"contact_filter");
	ros::NodeHandle nh;
	sub = nh.subscribe<LinkStates>("/gazebo/link_states", 1, callback);
	loc_contact_pub = nh.advertise<LinkStatesStamped>("/robbie/LinkStatesStamped", 1);
	msg.header.frame_id = "odom";
	rate.reset(new Rate(200));
	ros::spin();
	return 0;
}

}



int main(int argc, char** argv)
{
	return ros::main(argc, argv);
}

