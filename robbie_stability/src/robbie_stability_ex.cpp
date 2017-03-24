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
#include <math.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

namespace ros{
class StabilityPublisher{
private:
	NodeHandle nh_com;
	NodeHandle nh_con;
	Publisher pub_com_;
	Publisher pub_ssm;
	Publisher pub_l_mid_2;
	Publisher pub_l_mid;
	Subscriber contact_sub;
	boost::shared_ptr<boost::thread> tf_thread;
	Time time;
	std::unique_ptr<Rate> rate;
	std::unique_ptr<AsyncSpinner> async;
	geometry_msgs::PointStamped com;

	class Position{
	public:
		double x;
		double y;
		double z;

		void operator()(double x, double y, double z){
			this->x = x;
			this->y = y;
			this->z = z;
		}

		void operator()(Position l, Position r){
			this->x = (l.x + r.x)/2; 
			this->y = (l.y + r.y)/2; 
			this->z = 0;// std::sqrt(std::pow((l.x - com.point.x), 2) + std::pow((upper.y - com.point.y), 2));		
		}

	};

	Position shank_coords;
	Position lhm_coords;
	Position stabilizer_coords;
	Position optimum_postion;



	void load_tfs(){
		tf::TransformListener listener;

		geometry_msgs::PointStamped shank_foot_print_msg;
		geometry_msgs::PointStamped thigh_link_msg;
		geometry_msgs::PointStamped torso_link_msg;
		geometry_msgs::PointStamped lhm_link_msg;
		geometry_msgs::PointStamped lhm_wheel_left_link_msg;
		geometry_msgs::PointStamped lhm_wheel_right_link_msg;
		geometry_msgs::PointStamped shoulder_left_link_msg;
		geometry_msgs::PointStamped arm_left_link_msg;
		geometry_msgs::PointStamped shoulder_right_link_msg;
		geometry_msgs::PointStamped arm_right_link_msg;
		geometry_msgs::PointStamped stab_link_msg;
		geometry_msgs::PointStamped stab_wheel_msg;
		geometry_msgs::PointStamped wheel_left_link_msg;
		geometry_msgs::PointStamped wheel_right_link_msg;

		geometry_msgs::PointStamped tf_com;

		pub_com_ = nh_com.advertise<geometry_msgs::PointStamped>("/robbie/com", 10);

		shank_foot_print_msg.point.x = 0.163037;
		shank_foot_print_msg.point.y = 0;
		shank_foot_print_msg.point.z = 0.159341;

		thigh_link_msg.point.x = 0;
		thigh_link_msg.point.y = 0;
		thigh_link_msg.point.z = 0.1815;

		torso_link_msg.point.x = 0;
		torso_link_msg.point.y = 0;
		torso_link_msg.point.z = 0.192;

		lhm_link_msg.point.x = 0;
		lhm_link_msg.point.y = 0;
		lhm_link_msg.point.z = 0.02;

		lhm_wheel_left_link_msg.point.x = 0;
		lhm_wheel_left_link_msg.point.y = 0;
		lhm_wheel_left_link_msg.point.z = 0;

		lhm_wheel_right_link_msg.point.x = 0;
		lhm_wheel_right_link_msg.point.y = 0;
		lhm_wheel_right_link_msg.point.z = 0;

		shoulder_left_link_msg.point.x = 0;
		shoulder_left_link_msg.point.y = 0.16;
		shoulder_left_link_msg.point.z = -0.15;

		arm_right_link_msg.point.x = 0;
		arm_right_link_msg.point.y = 0;
		arm_right_link_msg.point.z = -0.1325;

		shoulder_right_link_msg.point.x = 0;
		shoulder_right_link_msg.point.y = -0.16;
		shoulder_right_link_msg.point.z = -0.15;

		arm_left_link_msg.point.x = 0;
		arm_left_link_msg.point.y = 0;
		arm_left_link_msg.point.z = -0.1325;

		stab_link_msg.point.x = 0;
		stab_link_msg.point.y = 0;
		stab_link_msg.point.z = 0.04625;

		stab_wheel_msg.point.x = 0;
		stab_wheel_msg.point.y = 0;
		stab_wheel_msg.point.z = 0;

		wheel_left_link_msg.point.x = 0;
		wheel_left_link_msg.point.y = 0;
		wheel_left_link_msg.point.z = 0;

		wheel_right_link_msg.point.x = 0;
		wheel_right_link_msg.point.y = 0;
		wheel_right_link_msg.point.z = 0;


		shank_foot_print_msg.header.frame_id = "shank_footprint";
		thigh_link_msg.header.frame_id = "thigh_link";
		torso_link_msg.header.frame_id = "torso_link";
		lhm_link_msg.header.frame_id = "lhm_link";
		lhm_wheel_right_link_msg.header.frame_id = "lhm_wheel_right_link";
		lhm_wheel_left_link_msg.header.frame_id = "lhm_wheel_left_link";
		shoulder_left_link_msg.header.frame_id = "shoulder_left_link";
		shoulder_right_link_msg.header.frame_id = "shoulder_right_link";
		arm_left_link_msg.header.frame_id = "arm_left_link";
		arm_right_link_msg.header.frame_id = "arm_right_link";
		stab_link_msg.header.frame_id = "stab_link";
		stab_wheel_msg.header.frame_id = "stab_wheel";
		wheel_left_link_msg.header.frame_id = "wheel_left_link";
		wheel_right_link_msg.header.frame_id = "wheel_right_link";

		com.header.frame_id = "odom";


		double total_mass = 7 + 2 + 16 + 2.25 + 0.2 + 0.2 + 0.25 + 0.25 + 0.25 +0.25 +0.04625 + 0.75 +0.25 + 0.25;



		while(nh_com.ok()){
			time = Time(0);
			shank_foot_print_msg.header.stamp = time;
			thigh_link_msg.header.stamp = time;
			shank_foot_print_msg.header.stamp = time;
			thigh_link_msg.header.stamp = time;
			torso_link_msg.header.stamp = time;
			lhm_link_msg.header.stamp = time;
			lhm_wheel_right_link_msg.header.stamp = time;
			lhm_wheel_left_link_msg.header.stamp = time;
			shoulder_left_link_msg.header.stamp = time;
			shoulder_right_link_msg.header.stamp = time;
			arm_left_link_msg.header.stamp = time;
			arm_right_link_msg.header.stamp = time;
			stab_link_msg.header.stamp = time;
			stab_wheel_msg.header.stamp = time;
			wheel_left_link_msg.header.stamp = time;
			wheel_right_link_msg.header.stamp = time;

			com.point.x=0;
			com.point.y=0;
			com.point.z=0;
			try{
				listener.transformPoint("odom", shank_foot_print_msg, tf_com);
				com.point.x+=tf_com.point.x*7;
				com.point.y+=tf_com.point.y*7;
				com.point.z+=tf_com.point.z*7;
				listener.transformPoint("odom", thigh_link_msg, tf_com);
				com.point.x+=tf_com.point.x*2;
				com.point.y+=tf_com.point.y*2;
				com.point.z+=tf_com.point.z*2;
				listener.transformPoint("odom", torso_link_msg, tf_com);
				com.point.x+=tf_com.point.x*16;
				com.point.y+=tf_com.point.y*16;
				com.point.z+=tf_com.point.z*16;
				listener.transformPoint("odom", lhm_link_msg, tf_com);
				com.point.x+=tf_com.point.x*2.25;
				com.point.y+=tf_com.point.y*2.25;
				com.point.z+=tf_com.point.z*2.25;
				listener.transformPoint("odom", lhm_wheel_right_link_msg, tf_com);
				com.point.x+=tf_com.point.x*0.2;
				com.point.y+=tf_com.point.y*0.2;
				com.point.z+=tf_com.point.z*0.2;
				listener.transformPoint("odom", lhm_wheel_left_link_msg, tf_com);
				com.point.x+=tf_com.point.x*0.2;
				com.point.y+=tf_com.point.y*0.2;
				com.point.z+=tf_com.point.z*0.2;
				listener.transformPoint("odom", shoulder_left_link_msg, tf_com);
				com.point.x+=tf_com.point.x*0.25;
				com.point.y+=tf_com.point.y*0.25;
				com.point.z+=tf_com.point.z*0.25;
				listener.transformPoint("odom", arm_left_link_msg, tf_com);
				com.point.x+=tf_com.point.x*0.25;
				com.point.y+=tf_com.point.y*0.25;
				com.point.z+=tf_com.point.z*0.25;
				listener.transformPoint("odom", shoulder_right_link_msg, tf_com);
				com.point.x+=tf_com.point.x*0.25;
				com.point.y+=tf_com.point.y*0.25;
				com.point.z+=tf_com.point.z*0.25;
				listener.transformPoint("odom", arm_right_link_msg, tf_com);
				com.point.x+=tf_com.point.x*0.25;
				com.point.y+=tf_com.point.y*0.25;
				com.point.z+=tf_com.point.z*0.25;
				listener.transformPoint("odom", stab_link_msg, tf_com);
				com.point.x+=tf_com.point.x*0.04625;
				com.point.y+=tf_com.point.y*0.04625;
				com.point.z+=tf_com.point.z*0.04625;
				listener.transformPoint("odom", stab_wheel_msg, tf_com);
				com.point.x+=tf_com.point.x*0.75;
				com.point.y+=tf_com.point.y*0.75;
				com.point.z+=tf_com.point.z*0.75;
				listener.transformPoint("odom", wheel_left_link_msg, tf_com);
				com.point.x+=tf_com.point.x*0.25;
				com.point.y+=tf_com.point.y*0.25;
				com.point.z+=tf_com.point.z*0.25;
				listener.transformPoint("odom", wheel_right_link_msg, tf_com);
				com.point.x+=tf_com.point.x*0.25;
				com.point.y+=tf_com.point.y*0.25;
				com.point.z+=tf_com.point.z*0.25;
				com.point.x=com.point.x/total_mass;
				com.point.y=com.point.y/total_mass;
				com.point.z=com.point.z/total_mass;

				tf::StampedTransform transform;
				try{
					listener.lookupTransform("odom", "shank_link", ros::Time(0), transform);
					shank_coords(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
					listener.lookupTransform("odom", "lhm_link", ros::Time(0), transform);
					lhm_coords(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
					listener.lookupTransform("odom", "stab_wheel", ros::Time(0), transform);
					stabilizer_coords(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
				}
				catch(tf::TransformException ex){
					ROS_ERROR("%s", ex.what());
					ros::Duration(1.0).sleep();
				}

				// ROS_INFO("COM(%f, %f, %f)", 
				// 	com.point.x, com.point.y, com.point.z);
				pub_com_.publish(com);
			}
			catch(tf::TransformException ex){
				ROS_ERROR("%s", ex.what());
				ros::Duration(1.0).sleep();
			}
			rate->sleep();

		}
		
	}


	double static_stability_margin(Position upper, Position lower){
		double d1 = std::sqrt(std::pow((upper.x - com.point.x), 2) + std::pow((upper.y - com.point.y), 2));
		double d2 = std::sqrt(std::pow((lower.x - com.point.x), 2) + std::pow((lower.y - com.point.y), 2));
		if(d1 < d2)
			return d1;
		return d2;
	}

	

	void calculate_stability(robbie_stability::Contact msg){
		if(msg.stabilizer && msg.lhm){
			std_msgs::Float32 msg;
			std_msgs::Float64 msg_opt;
			std_msgs::Float64 msg_opt2;

			msg.data = static_stability_margin(stabilizer_coords, lhm_coords);
			optimum_postion(shank_coords, lhm_coords);
			msg_opt.data =  std::sqrt(std::pow((optimum_postion.x - com.point.x), 2) + std::pow((optimum_postion.y - com.point.y), 2));
			// msg_opt2.data = std::sqrt(std::pow((optimum_postion.x - com.point.x), 2) + std::pow((optimum_postion.y - com.point.y), 2));
			pub_l_mid.publish(msg_opt);
			// pub_l_mid_2.publish(msg_opt2);
			pub_ssm.publish(msg);
			return;
		}
		else if(msg.shank && msg.lhm){
			std_msgs::Float32 msg;
			std_msgs::Float64 msg_opt;
			std_msgs::Float64 msg_opt2;

			msg.data = static_stability_margin(shank_coords, lhm_coords);
			optimum_postion(shank_coords, lhm_coords);
			msg_opt.data =  std::sqrt(std::pow((optimum_postion.x - com.point.x), 2) + std::pow((optimum_postion.y - com.point.y), 2));
			// msg_opt2.data = std::sqrt(std::pow((optimum_postion.x - com.point.x), 2) + std::pow((optimum_postion.y - com.point.y), 2));
			pub_l_mid.publish(msg_opt);
			// pub_l_mid_2.publish(msg_opt2);
			pub_ssm.publish(msg);
			return;
		}
		else if(msg.shank && msg.stabilizer){
			std_msgs::Float32 msg;
			std_msgs::Float64 msg_opt;
			std_msgs::Float64 msg_opt2;

			msg.data = static_stability_margin(shank_coords, stabilizer_coords);
			optimum_postion(shank_coords, stabilizer_coords);
			msg_opt.data =  std::sqrt(std::pow((optimum_postion.x - com.point.x), 2) + std::pow((optimum_postion.y - com.point.y), 2));
			// msg_opt2.data = std::sqrt(std::pow((optimum_postion.x - com.point.x), 2) + std::pow((optimum_postion.y - com.point.y), 2));
			pub_l_mid.publish(msg_opt);
			// pub_l_mid_2.publish(msg_opt2);
			pub_ssm.publish(msg);
			return;
		}
		else{
			// msg.data = 0;
			// msg.opt.data = 1
		}

	}




public:

	~StabilityPublisher(){
	}

	StabilityPublisher(){
		nh_com = NodeHandle("~");
		nh_con = NodeHandle("~");
		rate.reset(new Rate(20));
		pub_ssm = nh_con.advertise<std_msgs::Float32>("/robbie/ssm", 10, true);
		pub_l_mid = nh_con.advertise<std_msgs::Float64>("/robbie/ssm_delta", 10, true);
		// pub_l_mid_2 = nh_con.advertise<std_msgs::Float64>("/robbie/ssm_delta2", 10, true);
		tf_thread.reset(new boost::thread(boost::bind(&StabilityPublisher::load_tfs, this)));
		contact_sub = nh_con.subscribe("/robbie/contact", 10, &StabilityPublisher::calculate_stability, this);
		async.reset(new AsyncSpinner(0));
		async->start();
		waitForShutdown();
	}


};


int main(int argc, char** argv)
{
	init(argc, argv,"StabilityPub");
	StabilityPublisher s;
	return 0;
}

}



int main(int argc, char** argv)
{
	return ros::main(argc, argv);
}


