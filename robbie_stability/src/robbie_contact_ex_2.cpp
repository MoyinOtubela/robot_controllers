#include <ros/ros.h>
#include <memory>
#include <gazebo/transport/transport.hh>
#include <gazebo_msgs/ContactsState.h>
#include <robbie_stability/Contact.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


namespace ros{

using namespace message_filters;
using namespace gazebo_msgs;


void callback(const ContactsState& stab, const ContactsState& left_wheel, const ContactsState& right_wheel,const ContactsState& lhm_left_wheel, const ContactsState& lhm_right_wheel){
	if(stab.states.size() > 0){
		std::cout << stab.states[0].collision1_name << std::endl;
		std::cout << stab.states[0].collision2_name << std::endl;
	}
	// ROS_INFO('%s', *stab);
}


int main(int argc, char** argv)
{
	init(argc, argv,"contact_filter");
	ros::NodeHandle nh;
	ros::Publisher pub_contact;
	message_filters::Subscriber<ContactsState> stab_sub(nh, "/robbie/contact/stab_wheel",1);
	message_filters::Subscriber<ContactsState> left_wheel_sub(nh, "/robbie/contact/left_wheel",1);
	message_filters::Subscriber<ContactsState> right_wheel_sub(nh, "/robbie/contact/right_wheel",1);
	message_filters::Subscriber<ContactsState> lhm_left_wheel_sub(nh, "/robbie/contact/lhm_left_wheel",1);
	message_filters::Subscriber<ContactsState> lhm_right_wheel_sub(nh, "/robbie/contact/lhm_right_wheel",1);
	TimeSynchronizer<ContactsState, ContactsState, ContactsState, ContactsState, ContactsState> sync(stab_sub, left_wheel_sub, right_wheel_sub, lhm_left_wheel_sub, lhm_right_wheel_sub,10);
	sync.registerCallback(&callback);
	ros::spin();
	return 0;
}

}



int main(int argc, char** argv)
{

	return ros::main(argc, argv);
}


	// Timer timer_1 = nh.createTimer(ros::Duration(0.05), shank_timer, false);
	// Timer timer_2 = nh.createTimer(ros::Duration(0.5), stabilizer_timer, false);
	// Timer timer_3 = nh.createTimer(ros::Duration(0.05), lhm_timer, false);
	// Timer timer_4 = nh.createTimer(ros::Duration(0.05), contact_publisher, false);
	// async.reset(new AsyncSpinner(0));
	// async->start();
