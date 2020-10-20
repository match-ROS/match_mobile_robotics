///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////DANGEROUS PROTOTYPE////////////////////////////////////////////////////////////////////////////////
////////////////////////////DANGEROUS PROTOTYPE////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <ur_controllers_extended/compliant_axis_vel_controller.h>

namespace ur_controllers_extended{


bool CompliantAxisVelController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &n)
{
    if(!n.getParam("joint_stiffness", this->stiffness_))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << "joint_stiffness" << "' (namespace: " << n.getNamespace() << ").");
    }
    if(!n.getParam("p_gain", this->p_gain_))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << "p_gain" << "' (namespace: " << n.getNamespace() << ").");
    }
    if(!n.getParam("d_gain", this->d_gain_))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << "d_gain" << "' (namespace: " << n.getNamespace() << ").");
    }
    if(!n.getParam("theta", this->theta_))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << "theta" << "' (namespace: " << n.getNamespace() << ").");
    }
    // List of controlled joints
    if(!n.getParam("joints", this->joint_names_))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << "joints" << "' (namespace: " << n.getNamespace() << ").");
      return false;
    }
    if(this->joint_names_.size() == 0){
      ROS_ERROR_STREAM("List of joint names is empty.");
      return false;
    }
    for(unsigned int i=0; i<this->joint_names_.size(); i++)
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

    std::string direction;
    if(!n.getParam("direction", direction))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << "direction" << "' (namespace: " << n.getNamespace() << ").");
      return false;
    }  
    else
    {
      if(direction=="x")
      {
        this->direction_=Direction::x_axis;
      }
      else if(direction=="y")
      {
        this->direction_=Direction::y_axis;
      }
      else if(direction=="z")
      {
        this->direction_=Direction::z_axis;
      }
      else
      {
        ROS_ERROR_STREAM("Wrong torque direction specified! Possibilities are: 'x', 'y', 'z'");
        return false;
      }
      
    }

    std::string topic_name;
    if(!n.getParam("topic_name", topic_name))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << "topic_name" << "' (namespace: " << n.getNamespace() << ").");
      return false;
    } 
    this->wrench_sub_=n.subscribe(topic_name,1,&CompliantAxisVelController::wrenchCallback,this);
    this->config_server_.setCallback(boost::bind(&CompliantAxisVelController::dynConfigcallback,this,_1,_2));     

    this->time_old_=ros::Time::now();
    this->torque_=0.0;
    this->acceleration_=0.0;
    return true;

}

void CompliantAxisVelController::starting(const ros::Time& time)
{
  //Holding current position
  for(unsigned int i=0; i<this->joints_.size(); i++)
  {
    this->joints_[i].setCommand(0.0);
    
  }
  this->position_equi_=this->joints_.back().getPosition();
}



void CompliantAxisVelController::stopping(const ros::Time& time)
{}


void CompliantAxisVelController::update(const ros::Time& time, const ros::Duration& period)
{ 
  double d_M=0.0;
  if(this->d_time_.toSec()>0.0)
  {
    d_M=(this->torque_-this->torque_old_)/this->d_time_.toSec();
  }  
 
  this->vel_old_=this->joints_.back().getVelocity();
  double vel=this->p_gain_*this->torque_+this->d_gain_*d_M;
  this->joints_.back().setCommand(vel);
}

void CompliantAxisVelController::wrenchCallback(geometry_msgs::WrenchStamped msg)
{
  this->d_time_=msg.header.stamp-this->time_old_;
  if(this->d_time_.toSec()>0.0)
  {
    this->time_old_=msg.header.stamp;
    this->torque_old_=this->torque_;
  }
 
  double torque_measure=0.0;
  switch (this->direction_)
  {   
    case Direction::x_axis:
      torque_measure=msg.wrench.torque.x;
      break;
    case Direction::y_axis:
      torque_measure=msg.wrench.torque.y;
      break;
    case Direction::z_axis:
      torque_measure=msg.wrench.torque.z;
      break;  
  default:
    ROS_ERROR_STREAM("Direction of torque is not specified!");
    throw std::runtime_error("Direction of torque is not specified!");
    break;
  }  
  this->torque_=torque_measure-this->acceleration_*this->theta_;
}
void CompliantAxisVelController::dynConfigcallback(ur_controllers_extended::PDConfig &config, uint32_t level) {
    ROS_INFO_STREAM("Reconfigure Request: "<<config.p<<"/t"<<config.d);
    this->d_gain_=config.d;
    this->p_gain_=config.p;
    this->theta_=config.theta;
}

}//namespace

PLUGINLIB_EXPORT_CLASS(ur_controllers_extended::CompliantAxisVelController,
                       controller_interface::ControllerBase)
