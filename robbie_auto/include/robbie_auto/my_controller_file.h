#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

namespace my_controller_ns{

class MyControllerClass: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
private:

public:
  virtual bool init(hardware_interface::EffortJointInterface* hw,
                   ros::NodeHandle &n);
  virtual void starting();
  virtual void update(const ros::Time& time, const ros::Duration& period);
  virtual void stopping();
};
} 