#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_sequencer.h"
#include <geometry_msgs/Point32.h>
#include "sensor_msgs/point_cloud_conversion.h"



typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class KinectProcessor{
public:
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Subscriber sub_;
	std::string frame;
	std::string source;
	std::string topic;
	tf::TransformListener listener_;
	ros::Publisher step_pub_;
	ros::Publisher hole_pub_;

	struct pose{
		double x_;
		double y_;
		double z_;
		pose(double x, double y, double z):
		x_(x), y_(y), z_(z)
		{

		}
		pose(pcl::PointXYZ pt){
			this->x_=pt.x;
			this->x_=pt.y;
			this->x_=pt.z;
		}
	};

	KinectProcessor(ros::NodeHandle n):
		n_(n)
	{
		n_.getParam("source", this->source);
		n_.getParam("frame", this->frame);
		n_.getParam("topic", this->topic);

	    sub_ = n_.subscribe<PointCloud>(source, 1, boost::bind(&KinectProcessor::callback, this, _1));
	    pub_ = n_.advertise<sensor_msgs::PointCloud2>(topic,1);
		ROS_INFO("KinectProcessor initialized!");
	    step_pub_ = n_.advertise<geometry_msgs::Point32>("step_topic",1);
	    hole_pub_ = n_.advertise<geometry_msgs::Point32>("hole_topic",1);
		// n_.subscribe<PointCloud>(from_topic_, 1, &callback);
	}

	void process(){

	}

	// static bool compare_height(const pose& l, const pose& r){
	// 	return l.z_ < r.z_;
	// }  
	static bool compare_height(const geometry_msgs::Point32& l, const geometry_msgs::Point32& r){
		return l.z < r.z;
	}  


	// static bool compare_height(const pose& l, const pose& r){
	// 	return l.z_ < r.z_;
	// }  

	void callback(const PointCloud::ConstPtr& msg){
		PointCloud cloud;
	    try
	    {
	        pcl_ros::transformPointCloud(
	          this->frame,*msg, cloud,this->listener_);
	    }
	    catch (tf::TransformException& e)
	    {
	        std::cout << e.what();
	        return;
	    }
	    
		

		sensor_msgs::PointCloud2 msg_;

		pcl::toROSMsg(cloud, msg_);

		sensor_msgs::PointCloud points;
		sensor_msgs::convertPointCloud2ToPointCloud(msg_, points);

		// BOOST_FOREACH (const pcl::PointXYZ& pt, cloud.points){
		// 	// printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);		
		// 	points.push_back(pt);
		// }
				// std::vector<pose> points;
		// sensor_msgs::PointCloud points;
		// BOOST_FOREACH (const geometry_msgs::Point32& pt, cloud.points){
		// 	// printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);		
		// 	points.push_back(pt);
		// }
		if(points.points.size()>1){
			std::sort(points.points.begin(), points.points.end(), KinectProcessor::compare_height);
		    step_pub_.publish(*points.points.begin());
		    hole_pub_.publish(*points.points.end());
		}
		// ROS_INFO("Shank H Height @  %g", points.points.begin()->z);

		// ROS_INFO("Shank L Height @  %g", points.points.end()->z);

	    pub_.publish(msg_);


	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pointcloud_processor");
	ros::NodeHandle n("~");
	KinectProcessor s(n);
	ros::spin();
	/* code */
	return 0;
}

		// message_filters::Subscriber<PointCloud> sub_(n, source, 10);
		// // message_filters::TimeSequencer<PointCloud> seq_(sub_, ros::Duration(0.1), ros::Duration(0.01), 10);
		// tf::MessageFilter<PointCloud> pcl_notifier_(sub_, listener_, frame, 1);
		// pcl_notifier_.registerCallback(
		// 	boost::bind(&KinectProcessor::callback, this, _1));
		// pcl_notifier_.setTolerance(ros::Duration(0.1));
