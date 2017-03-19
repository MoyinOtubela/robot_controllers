#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <boost/foreach.hpp>
#include <geometry_msgs/Point32.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/io.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>

// #include <pcl_ros/point_cloud.h>
// #include <pcl_ros/impl/transforms.hpp>
#include <pcl/point_types.h>
// #include <pcl/cloud_iterator.h>

// finish off

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

pcl::VoxelGrid<PointT> sor;
PointCloud::Ptr cloud_filtered (new PointCloud);
PointCloud::Ptr cloud (new PointCloud);


class LaserScanToPointCloud{

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Subscriber sub;
  std::string base_link;
  ros::Publisher scan_pub_;
  ros::Publisher step_pub_;
  ros::Publisher hole_pub_;

  LaserScanToPointCloud(ros::NodeHandle n, std::string base_scan, std::string base_link, std::string pointcloud_topic) : 
    n_(n),
    laser_sub_(n_, base_scan, 10),
    laser_notifier_(laser_sub_,listener_, base_link, 10)
  {
    this->base_link = base_link;
    laser_notifier_.registerCallback(
      boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.001));
    std::string step_topic;
    std::string hole_topic;
    // std::string filtered_topic;
    n_.getParam("step_topic", step_topic);
    n_.getParam("hole_topic", hole_topic);
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud2>(pointcloud_topic,1);
    step_pub_ = n_.advertise<geometry_msgs::Point32>(step_topic,1);
    hole_pub_ = n_.advertise<geometry_msgs::Point32>(hole_topic,1);

    ROS_INFO("LaserScan initialized!");
  }

  // static bool compare_height(const PointT::iterator l, const PointT::iterator r){
  //   // return l.z < r.z;
  // }  
  static bool compare_height(const geometry_msgs::Point32& l, const geometry_msgs::Point32& r){
    return l.z < r.z;
  }  


  void process(sensor_msgs::PointCloud2 cloud_){
    // Detect step when z > threshold
    // Send x y location for following.. CA uses this to determine if obstacle or not
    pcl::moveFromROSMsg(cloud_, *cloud);
    sor.setInputCloud(cloud);
    // sor.setLeafSize(0.9f,0.9f,0.9f);
    sor.filter(*cloud_filtered);
    sensor_msgs::PointCloud2 cloud_msg;
    // pcl_conversions::copyPCLPointCloud2MetaData(processed_cloud, cloud_msg);
    pcl::toROSMsg(*cloud_filtered, cloud_msg);
    sensor_msgs::PointCloud msg;
    sensor_msgs::convertPointCloud2ToPointCloud(cloud_msg, msg);
    std::sort(msg.points.begin(), msg.points.end(), LaserScanToPointCloud::compare_height);
    
    step_pub_.publish(*msg.points.begin());
    hole_pub_.publish(*msg.points.end());
    ROS_INFO("Laser H Height @  %g", msg.points.begin()->z);
    ROS_INFO("Laser L Height @  %g", msg.points.end()->z);

    // BOOST_FOREACH (geometry_msgs::Point32 pt, cloud.points)
    //  printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);    

  }


  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {

    sensor_msgs::PointCloud2 cloud;

    try
    {
        projector_.transformLaserScanToPointCloud(
          this->base_link,*scan_in, cloud,listener_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }

    process(cloud);
    scan_pub_.publish(cloud);

  }
};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "laser_to_pointcloud");
  ros::NodeHandle n("~");
  std::string base_link;
  std::string base_scan;
  std::string pointcloud_topic;
  n.getParam("base_link", base_link);
  n.getParam("base_scan", base_scan);
  n.getParam("pointcloud_topic", pointcloud_topic);

  LaserScanToPointCloud lstopc(n, base_scan, base_link, pointcloud_topic);
  
  ros::spin();
  
  return 0;
}


    // pcl_conversions::toPCL(ros::Time::now(), msg.header.stamp);
     // msg.header.stamp = ros::Time::now();
    // pcl::toPCLPointCloud2(*cloud_filtered, converted_cloud);

    // sensor_msgs::PointCloud cloud;
    // sensor_msgs::convertPointCloudToPointCloud2(cloud_, cloud);
    // pcl::PCLPointCloud2 dummy;
    // pcl::PCLPointCloud2 cloud_filtered;
    // pcl::PCLPointCloud2ConstPtr cloudPtr(&dummy);
    // pcl_conversions::toPCL(cloud, dummy);
    // vg.setInputCloud(cloudPtr);
    // vg.setLeafSize(0.1f,0.1f,0.1f);
    // vg.filter(cloud_filtered);
    // sensor_msgs::PointCloud2 cloud2;
    // pcl_conversions::fromPCL(cloud_filtered, cloud2);
    // pcl::moveFromROSMsg(cloud, dummy);

    // std::sort(b, e, LaserScanToPointCloud::compare_height);
    // std::sort(cloud->begin(), cloud->end(), LaserScanToPointCloud::compare_height);
    // pcl::PCLPointCloud2 processed_cloud;
    // pcl::toPCLPointCloud2(*cloud_filtered, processed_cloud);
    // pcl_conversions::toPCL(ros::Time::now(), processed_cloud.header.stamp);
    // sub = n_.subscriber<geometry_msgs::Point32>(hole_topic,1, LaserScanToPointCloud::scanCallback);
