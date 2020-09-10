#include <ur_controllers_extended/compliant_axis_controller.h>

namespace ur_controllers_extended{


bool CompliantAxisController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n)
{
    if(!n.getParam("joint_stiffness", this->stiffness_))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << "joint_stiffness" << "' (namespace: " << n.getNamespace() << ").");
    }

    // List of controlled joints
    std::string param_name = "joints";
    if(!n.getParam(param_name, this->joint_names_))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
      return false;
    }
    this->n_joints_ = this->joint_names_.size();

    if(this->n_joints_ == 0){
      ROS_ERROR_STREAM("List of joint names is empty.");
      return false;
    }
    for(unsigned int i=0; i<this->n_joints_; i++)
    {
      try
      {
        this->joints_.push_back(hw->getHandle(this->joint_names_[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException& e)
      {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
      }
    }
    this->measure_=true;
    return true;

}

void CompliantAxisController::starting(const ros::Time& time)
{
  //Holding current position
  for(unsigned int i=0; i<this->n_joints_; i++)
  {
    this->joints_[i].setCommand(this->joints_[i].getPosition());
    
  }
  this->position_equi_=this->joints_.back().getPosition();
}
void CompliantAxisController::stopping(const ros::Time& time)
{}
void CompliantAxisController::update(const ros::Time& time, const ros::Duration& period)
{
  if(std::abs(this->joints_.back().getVelocity())<0.01)
  {
    this->last_effort_=this->joints_.back().getEffort();
    if(this->stiffness_>0.0)
    {
      this->joints_.back().setCommand(position_equi_-this->last_effort_/this->stiffness_);
    }  
  }
  ROS_INFO_STREAM(this->last_effort_);
}

}//namespace

PLUGINLIB_EXPORT_CLASS(ur_controllers_extended::CompliantAxisController,
                       controller_interface::ControllerBase)
