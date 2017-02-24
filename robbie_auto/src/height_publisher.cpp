#include <ros/ros.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>
#include <memory>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <math.h>

namespace ros{
	class HeightPublisher{
	private:
		boost::shared_ptr<Rate> rate_2;
		Publisher pub_;
		Time time;
		std_msgs::Float64 msg;
		Subscriber sub;

	public:

		HeightPublisher(int argc, char** argv){
			init(argc, argv, "height_publisher");
			NodeHandle nh_;
			rate_2 = boost::make_shared<Rate>(10);
			tf::StampedTransform transform;
			tf::TransformListener listener;
			pub_ = nh_.advertise<std_msgs::Float64>("/robbie/height",10);
			while (nh_.ok()){
				try{
					listener.lookupTransform("odom","camera_link", Time(0), transform);
					msg.data = transform.getOrigin().z();
					ROS_INFO("Height = %.2f m", transform.getOrigin().z());
					pub_.publish(msg);
					rate_2->sleep();
				}
				catch(tf::TransformException &ex){
					ROS_ERROR("%s", ex.what());
					Duration(1.0).sleep();
					continue;
				}
			}
		}



	};	

	int main(int argc, char** argv){
		HeightPublisher s(argc, argv);
		return 0;
	}
}


int main(int argc, char** argv)
{
	return ros::main(argc, argv);
}
