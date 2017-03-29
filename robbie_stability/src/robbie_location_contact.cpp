#include <ros/ros.h>
#include <memory>
#include <gazebo/transport/transport.hh>
#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/LinkStates.h>
#include <robbie_stability/Contact.h>
#include <robbie_stability/ContactLinkStates.h>
#include <robbie_stability/LinkStatesStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/time_sequencer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include "boost/algorithm/string.hpp"


namespace ros{

using namespace message_filters;
using namespace gazebo_msgs;
using namespace robbie_stability;

// ContactLinkStates msg;
ros::Publisher pub;
ros::Subscriber sub;
std::unique_ptr<Rate> rate;
std::unique_ptr<Rate> update_rate;
ContactLinkStates msg;

int shank_footprint=-1;
int thigh_link;
int torso_link;
int lhm_link;
int lhm_wheel_left_link; 
int lhm_wheel_right_link;
int shoulder_left_link;
int arm_left_link;
int shoulder_right_link;
int arm_right_link;
int stab_link;
int stab_wheel;
int wheel_left_link;
int wheel_right_link;


void callback(const Contact& contact, const LinkStatesStamped& link_states){
	msg.contact = contact;
	msg.shank_footprint = link_states.states.pose[shank_footprint];
	msg.thigh_link = link_states.states.pose[thigh_link];
	msg.torso_link = link_states.states.pose[torso_link];
	msg.lhm_link = link_states.states.pose[lhm_link];
	msg.lhm_wheel_left_link = link_states.states.pose[lhm_wheel_left_link];
	msg.lhm_wheel_right_link = link_states.states.pose[lhm_wheel_right_link];
	msg.shoulder_left_link = link_states.states.pose[shoulder_left_link];
	msg.shoulder_right_link = link_states.states.pose[shoulder_right_link];
	msg.arm_left_link = link_states.states.pose[arm_left_link];
	msg.arm_right_link = link_states.states.pose[arm_right_link];
	msg.stab_link= link_states.states.pose[stab_link];
	msg.stab_wheel = link_states.states.pose[stab_wheel];
	msg.wheel_left_link = link_states.states.pose[wheel_left_link];
	msg.wheel_right_link = link_states.states.pose[wheel_right_link];
	// msg.states = link_states;
	pub.publish(msg);
	// rate->sleep();
}

void info_callback(LinkStates msg){
	if(msg.name.size()>0){
		for(int i = 0; i != msg.name.size(); ++i){
			// ROS_WARN("Waiting for topic /gazebo/link_states");
			// std::cout << msg.name.size() << endl;
			// std::cout << i << std::endl;

			if(boost::contains(msg.name[i].c_str(), "shank_footprint")){
				shank_footprint = i;
			}
			if(boost::contains(msg.name[i].c_str(), "thigh_link")){
				thigh_link = i;
			}
			if(boost::contains(msg.name[i].c_str(), "torso_link")){
				torso_link = i;
			}
			if(boost::contains(msg.name[i].c_str(), "lhm_link")){
				lhm_link = i;
			}
			if(boost::contains(msg.name[i].c_str(), "stab_link")){
				stab_link = i;
			}
			if(boost::contains(msg.name[i].c_str(), "stab_wheel")){
				stab_wheel = i;
			}
			if(boost::contains(msg.name[i].c_str(), "wheel_left_link")){
				wheel_left_link = i;
			}
			if(boost::contains(msg.name[i].c_str(), "wheel_right_link")){
				wheel_right_link = i;
			}
			if(boost::contains(msg.name[i].c_str(), "lhm_wheel_left_link")){
				lhm_wheel_left_link = i;
			}
			if(boost::contains(msg.name[i].c_str(), "lhm_wheel_right_link")){
				lhm_wheel_right_link = i;
			}
			if(boost::contains(msg.name[i].c_str(), "shoulder_left_link")){
				shoulder_left_link = i;
			}
			if(boost::contains(msg.name[i].c_str(), "shoulder_right_link")){
				shoulder_right_link = i;
			}
			if(boost::contains(msg.name[i].c_str(), "arm_left_link")){
				arm_left_link = i;
			}
			if(boost::contains(msg.name[i].c_str(), "arm_right_link")){
				arm_right_link = i;
			}
		}
	}
	// update_rate->sleep();

}

int main(int argc, char** argv)
{
	init(argc, argv,"Contact_States_filter");
	ros::NodeHandle nh;
	pub = nh.advertise<robbie_stability::ContactLinkStates>("/robbie/LocationContact", 1);
	sub = nh.subscribe<LinkStates>("/gazebo/link_states", 1, &info_callback);
	while(shank_footprint==-1){
		ROS_WARN("Waiting for topic /gazebo/link_states");
		ros::Duration(1).sleep();
		ros::spinOnce();
	}

	sub.shutdown(); //Unregisters subscriber

	message_filters::Subscriber<Contact> contact_sub(nh, "/robbie/contact",1);
	message_filters::Subscriber<LinkStatesStamped> link_states_sub(nh, "/robbie/LinkStatesStamped",1);
	rate.reset(new Rate(100));
	// update_rate.reset(new Rate(0.01));
    // typedef sync_policies::ApproximateTime<Contact, LinkStatesStamped> MySyncPolicy;
    typedef sync_policies::ExactTime<Contact, LinkStatesStamped> MySyncPolicy;

	// TimeSynchronizer<Contact, LinkStatesStamped> sync(contact_sub, link_states_sub, 1);
	// TimeSequencer<Contact, LinkStatesStamped> sync(contact_sub, link_states_sub, ros::Duration(0.05), ros::Duration(0.01), 1);
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), contact_sub, link_states_sub);
	sync.registerCallback(&callback);
	ros::spin();
	return 0;
}

}



int main(int argc, char** argv)
{
	return ros::main(argc, argv);
}

// shank footprint 2
// thigh_link 3
// torso_link 4
// lhm_link 5
// lhm_wheel_left_link 6 
// lhm_wheel_right_link 7
// shoulder_left_link 8
// arm_left_link 9 
// shoulder_right_link 10
// arm_right_link 11 
// stab_link 12
// stab_wheel 13
// wheel_left_link 14
// wheel_right_link 15