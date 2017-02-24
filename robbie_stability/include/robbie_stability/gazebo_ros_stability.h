#ifndef __GAZEBO_ROS_STABILITY_PLUGIN_HH__
#define __GAZEBO_ROS_STABILITY_PLUGIN_HH__

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <iostream>

#include <tinyxml.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <ros/package.h>
#include <rosgraph_msgs/Clock.h>


// Services
#include "gazebo_msgs/GetWorldProperties.h"
#include "gazebo_msgs/GetModelProperties.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/GetLinkProperties.h"
#include "gazebo_msgs/SetLinkProperties.h"
#include "gazebo_msgs/SetLinkState.h"
#include "gazebo_msgs/GetLinkState.h"
// Topics
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/LinkState.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/LinkStates.h"

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

// For model pose transform to set custom joint angles
#include <ros/ros.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <boost/shared_ptr.hpp>

#include <boost/algorithm/string.hpp>

namespace gazebo{

class GazeboRosStabilityPlugin : public SystemPlugin{
public:
	GazeboRosStabilityPlugin();
	~GazeboRosStabilityPlugin();

	//methods
	void shutdownSignal();
  void Load(int argc, char** argv);
	void gazeboQueueThread();
	void loadGazeboRosStabilityPlugin(std::string world_name);
	void loadGazeboRosStabilityPlugin();
	void advertisePublishers();
  void onCOMConnect();
  void onCOMDisconnect();
  void publishRobotProperties();
  void publishSimTime();
  //algorithm variables


  std::vector<std::string> model_names;



	//member variables
  std::string robot_namespace_;
  bool stop_; 
  bool world_created_;
  bool physics_reconfigure_initialized_;
  bool plugin_loaded_;
  int                pub_link_states_connection_count_;
  int                pub_robot_states_connection_count_;
	int                pub_model_states_connection_count_;
	boost::mutex lock_;


	gazebo::physics::WorldPtr world_;

  gazebo::event::ConnectionPtr sigint_event_;
  boost::shared_ptr<boost::thread> physics_reconfigure_thread_;
  boost::shared_ptr<boost::thread> gazebo_callback_queue_thread_;
  boost::shared_ptr<ros::NodeHandle> nh_;
  boost::shared_ptr<ros::AsyncSpinner> async_ros_spin_;
  gazebo::event::ConnectionPtr time_update_event_;
  gazebo::event::ConnectionPtr pub_robot_states_event_;
  gazebo::event::ConnectionPtr load_gazebo_ros_stability_plugin_event_;
  gazebo::transport::NodePtr gazebonode_;


  ros::CallbackQueue gazebo_queue_;
  ros::Publisher pub_com_;
  ros::Publisher pub_clock_;
  int pub_clock_frequency_;
  gazebo::common::Time last_pub_clock_time_;
  // std::vector<gazebo::physics::LinkPtr> bodies;
  gazebo::physics::LinkPtr body;

  // std::vector<event::ConnectionPtr


  // ros::AdvertiseOptions pub_

};


}

#endif
