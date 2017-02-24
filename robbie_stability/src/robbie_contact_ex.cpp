#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <memory>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <map>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <robbie_stability/Contact.h>
#include "boost/algorithm/string.hpp"
namespace ros{
	// Need to find contacts
	// Assume by configuration?
	// Use position of stabiliser wheel and the shank footprint
	// If the lhm is down & stabilizer wheel is down.. use from these ends
	// If the lhm is down & stabilizer wheel is down.. use from these ends
	// If only the stabiler wheel is down and lhm is down.. use these ends
	// checking for contacts between lhm_link, stabilizer_wheel & shank_footprint

int stabilizer_wheel_count=0;
int shank_count=0;
int lhm_wheel_count=0;
Publisher pub_contact;
robbie_stability::Contact contact_msg;
std::unique_ptr<Rate> rate;
std::unique_ptr<Rate> rate2;
std::unique_ptr<Rate> rate3;
boost::shared_ptr<boost::thread> shank_thread;
boost::shared_ptr<boost::thread> stabilizer_thread;
boost::shared_ptr<boost::thread> lhm_thread;
boost::shared_ptr<boost::thread> publisher_thread;
// NodeHandle nh;
std::unique_ptr<AsyncSpinner> async;
std::unique_ptr<NodeHandle> nh;

void determine_contacts(ConstContactsPtr &msg){
	if(msg->contact_size()!=0){
		if(boost::contains(msg->contact(0).collision1().c_str(), "wheel_left_link")
			|| boost::contains(msg->contact(0).collision1().c_str(), "wheel_right_link")){
			shank_count+=1;
			// ROS_INFO("shank is in contact");
		}
		if(boost::contains(msg->contact(0).collision1().c_str(), "stab_wheel")){
			stabilizer_wheel_count+=1;
			// ROS_INFO("stabilizer is in contact");
		}
		if(boost::contains(msg->contact(0).collision1().c_str(), "lhm")){
			lhm_wheel_count+=1;
			// ROS_INFO("lhm is in contact");
		}
	}
	// rate->sleep();
}

void shank_timer(const ros::TimerEvent&){
	if(shank_count>=5){
		contact_msg.shank = true;
		ROS_INFO("shank_timer");
	}
	else{
		contact_msg.shank = false;
	}
	shank_count=0;
}

void stabilizer_timer(const ros::TimerEvent&){
	if(stabilizer_wheel_count>=2){
		contact_msg.stabilizer = true;
		ROS_INFO("stabilizer_timer");
	}
	else{
		contact_msg.stabilizer = false;
	}

	stabilizer_wheel_count=0;
}
void contact_publisher(const ros::TimerEvent&){
	pub_contact.publish(contact_msg);
}
void lhm_timer(const ros::TimerEvent&){
	if(lhm_wheel_count>=2){
		contact_msg.lhm = true;
		// ROS_INFO("lhm_timer");
	}
	else{
		contact_msg.lhm = false;
	}
	lhm_wheel_count=0;
}



void stabilizer_timer(void){
	while(nh->ok()){
		if(stabilizer_wheel_count>=2){
			contact_msg.stabilizer = true;
			// ROS_INFO("stabilizer_timer");
		}
		else{
			contact_msg.stabilizer = false;
		}

		stabilizer_wheel_count=0;
		rate3->sleep();
	}
}


void shank_timer(void){
	while(nh->ok()){
		if(shank_count>=5){
			contact_msg.shank = true;
			// ROS_INFO("shank_timer");
		}
		else{
			contact_msg.shank = false;
		}
		shank_count=0;
		rate2->sleep();
	}
}
void lhm_timer(void){
	while(nh->ok()){
		if(lhm_wheel_count>=2){
			contact_msg.lhm = true;
			// ROS_INFO("lhm_timer");
		}
		else{
			contact_msg.lhm = false;
		}
		lhm_wheel_count=0;
		rate2->sleep();
	}
}

void contact_publisher(void){
	while(nh->ok()){
		pub_contact.publish(contact_msg);
		rate2->sleep();
	}
}




int main(int argc, char** argv)
{
	init(argc, argv,"RobbieContact");
	gazebo::client::setup(argc, argv);
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();
	nh.reset(new NodeHandle("~"));// NodeHandle("~");

	rate.reset(new Rate(500));
	rate2.reset(new Rate(20));
	rate3.reset(new Rate(10));

	pub_contact = nh->advertise<robbie_stability::Contact>("/robbie/contact", 10);
	const std::string topic = "~/physics/contacts";
	gazebo::transport::SubscriberPtr sub = node->Subscribe(topic, determine_contacts, false);


	shank_thread.reset(new boost::thread(boost::bind(&shank_timer)));
	stabilizer_thread.reset(new boost::thread(boost::bind(&stabilizer_timer)));
	lhm_thread.reset(new boost::thread(boost::bind(&lhm_timer)));
	publisher_thread.reset(new boost::thread(boost::bind(&contact_publisher)));


	async.reset(new AsyncSpinner(0));
	async->start();
	waitForShutdown();
	gazebo::client::shutdown();
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
