#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>


void cb1(){
	while(ros::ok())
		ROS_INFO("callback 1");
}
void cb2(){
	while(ros::ok())
		ROS_INFO("callback 2");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "QueueTest");
	ros::NodeHandle nh_1("~");
	ros::NodeHandle nh_2("~");
	boost::shared_ptr<boost::thread> cb1_thread;
	boost::shared_ptr<boost::thread> cb2_thread;

	// ros::Timer timer_1 = nh_1.createTimer(ros::Duration(0.01), cb1, false);
	// ros::Timer timer_2 = nh_2.createTimer(ros::Duration(0.01), cb2, false);
	// shank_thread.reset(new boost::thread(boost::bind(&shank_timer)));

    ros::CallbackQueue queue;
    cb1_thread.reset(new boost::thread(boost::bind(cb1)));
    cb2_thread.reset(new boost::thread(boost::bind(cb2)));
    queue.callOne(ros::WallDuration(10));
	ros::AsyncSpinner spinner(0, &queue);

	spinner.start();
    ros::waitForShutdown();
	/* code */
	return 0;
}