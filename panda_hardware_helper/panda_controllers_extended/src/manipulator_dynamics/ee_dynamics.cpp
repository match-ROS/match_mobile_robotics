#include <panda_controllers_extended/manipulator_dynamics/ee_dynamics.h>  

EEDynamics::EEDynamics(ros::NodeHandle &nh):nh_(nh)                                            
{
    ros::NodeHandle param("~");
    double rate;
    if(!param.getParam("rate",rate))
    {
        ROS_ERROR_STREAM("EEDynamics could not find parameter rate!");
        throw std::runtime_error("EEDynamics could not find parameter rate!");
    }
    else
    {
        this->update_timer_=this->nh_.createTimer(ros::Rate(rate),&EEDynamics::update,this);
    }

    this->force_pub_=this->nh_.advertise<geometry_msgs::Wrench>("wrench",1);
    this->base_accel_sub_=this->nh_.subscribe("accel",1,&EEDynamics::accelCallback,this);
    this->force_.setZero();
    this->accel_.setZero();
    this->inertia_.setZero();
}

void EEDynamics::update(const ros::TimerEvent &)
{
    ROS_INFO_STREAM("IN");
    this->inertia_=this->updateInertia();
    this->force_=this->updateForce();
    


    geometry_msgs::WrenchStamped wrench;
    wrench.header.stamp=ros::Time::now();
    wrench.wrench.force.x=this->force_(0);
    wrench.wrench.force.y=this->force_(1);
    wrench.wrench.force.z=this->force_(2);
    wrench.wrench.torque.x=this->force_(3);
    wrench.wrench.torque.y=this->force_(4);
    wrench.wrench.torque.z=this->force_(5);
    ROS_ERROR_STREAM(this->force_);
    this->force_pub_.publish(wrench);

}
void EEDynamics::accelCallback(geometry_msgs::TwistStamped msg)
{
    this->accel_<<  msg.twist.linear.x,
                    msg.twist.linear.y,
                    msg.twist.linear.z,
                    msg.twist.angular.x,
                    msg.twist.angular.y,
                    msg.twist.angular.z;
}