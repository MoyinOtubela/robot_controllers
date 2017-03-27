#include <ros/ros.h>
#include <memory>
#include <gazebo/transport/transport.hh>
#include <gazebo_msgs/ContactsState.h>
#include <robbie_stability/Contact.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>


namespace ros{

using namespace message_filters;
using namespace gazebo_msgs;

robbie_stability::Contact contact_msg;
ros::Publisher pub_contact;
int stabilizer_count=0;
int shank_count=0;
int lhm_count=0;
std::unique_ptr<Rate> rate;

void callback(const ContactsState& stab, const ContactsState& left_wheel, const ContactsState& right_wheel,const ContactsState& lhm_left_wheel, const ContactsState& lhm_right_wheel){
	if(stab.states.size() > 0){
		contact_msg.stabilizer = true;
		contact_msg.stabilizer_on = stab.states[0].collision2_name;
		contact_msg.stabilizer_location.x = stab.states[0].contact_positions[0].x;
		contact_msg.stabilizer_location.y = stab.states[0].contact_positions[0].y;
		contact_msg.stabilizer_location.z = stab.states[0].contact_positions[0].z;
		stabilizer_count = 0;
	}
	else{
		stabilizer_count+=1;
		if (stabilizer_count>40){
			contact_msg.stabilizer = false;
		}
	}
	if(left_wheel.states.size() > 0 && right_wheel.states.size() > 0){
		contact_msg.shank = true;
		contact_msg.shank_on = left_wheel.states[0].collision2_name + " and " + right_wheel.states[0].collision2_name;
		contact_msg.shank_location.x = (left_wheel.states[0].contact_positions[0].x + right_wheel.states[0].contact_positions[0].x)/2;
		contact_msg.shank_location.y = (left_wheel.states[0].contact_positions[0].y + right_wheel.states[0].contact_positions[0].y)/2;
		contact_msg.shank_location.z = (left_wheel.states[0].contact_positions[0].z + right_wheel.states[0].contact_positions[0].z)/2;
		shank_count = 0;
	}
	else{
		shank_count+=1;
		if (shank_count>40){
			contact_msg.shank = false;
		}
	}
	if(lhm_left_wheel.states.size() > 0 && lhm_right_wheel.states.size() > 0){
		contact_msg.lhm = true;
		contact_msg.lhm_on = lhm_left_wheel.states[0].collision2_name + " and " + lhm_right_wheel.states[0].collision2_name;
		contact_msg.lhm_location.x = (lhm_left_wheel.states[0].contact_positions[0].x + lhm_right_wheel.states[0].contact_positions[0].x)/2;
		contact_msg.lhm_location.y = (lhm_left_wheel.states[0].contact_positions[0].y + lhm_right_wheel.states[0].contact_positions[0].y)/2;
		contact_msg.lhm_location.z = (lhm_left_wheel.states[0].contact_positions[0].z + lhm_right_wheel.states[0].contact_positions[0].z)/2;
		lhm_count = 0;
	}
	else{
		lhm_count+=1;
		if (lhm_count>40){
			contact_msg.lhm = false;
		}
	}
	contact_msg.header.stamp = ros::Time::now();
	pub_contact.publish(contact_msg);
	// rate->sleep();

}


int main(int argc, char** argv)
{
	init(argc, argv,"contact_filter");
	ros::NodeHandle nh;
	pub_contact = nh.advertise<robbie_stability::Contact>("/robbie/contact", 1);
	message_filters::Subscriber<ContactsState> stab_sub(nh, "/robbie/contact/stab_wheel",1);
	message_filters::Subscriber<ContactsState> left_wheel_sub(nh, "/robbie/contact/left_wheel",1);
	message_filters::Subscriber<ContactsState> right_wheel_sub(nh, "/robbie/contact/right_wheel",1);
	message_filters::Subscriber<ContactsState> lhm_left_wheel_sub(nh, "/robbie/contact/lhm_left_wheel",1);
	message_filters::Subscriber<ContactsState> lhm_right_wheel_sub(nh, "/robbie/contact/lhm_right_wheel",1);
	typedef sync_policies::ExactTime<ContactsState,ContactsState,ContactsState,ContactsState,ContactsState> MySyncPolicy;

	// rate.reset(new Rate(100));
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), stab_sub, left_wheel_sub, right_wheel_sub, lhm_left_wheel_sub, lhm_right_wheel_sub);
	// TimeSynchronizer<ContactsState, ContactsState, ContactsState, ContactsState, ContactsState> sync(stab_sub, left_wheel_sub, right_wheel_sub, lhm_left_wheel_sub, lhm_right_wheel_sub,10);
	sync.registerCallback(&callback);
	ros::spin();
	return 0;
}

}



int main(int argc, char** argv)
{
	return ros::main(argc, argv);
}

