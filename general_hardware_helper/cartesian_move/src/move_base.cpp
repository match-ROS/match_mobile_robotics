#include <cartesian_move/move_base.h>


MoveBase::MoveBase(ros::NodeHandle &nh):nh_(nh),listener_(buffer_)
{
    this->buffer_.setUsingDedicatedThread(true);
    ros::NodeHandle priv("~");
    if(!priv.getParam("ee_frame",this->ee_frame_))
    {
        throw std::runtime_error("Robots ee_frame parameter not found");
    }
    if(!priv.getParam("base_frame",this->base_frame_))
    {
        throw std::runtime_error("Robots base_frame parameter not found");
    }
    if(!priv.getParam("planning_frame",this->planning_frame_))
    {
        throw std::runtime_error("Robots planning_frame parameter not found");
    }

    double rate;
    if(!priv.getParam("rate",rate))
    {
        throw std::runtime_error("Rate parameter not found");
    }
    else
    {
        this->scope_timer_=this->nh_.createTimer(ros::Rate(rate),&MoveBase::scope,this);
    }
    
    this->pose_pub_=this->nh_.advertise<geometry_msgs::PoseStamped>("pose_out",10);
}

void MoveBase::init()
{
    try{
        geometry_msgs::TransformStamped trafo;
        trafo=this->buffer_.lookupTransform(this->base_frame_,this->ee_frame_,ros::Time(0.0),ros::Duration(10.0));
        tf2::fromMsg(trafo.transform,this->initial_pose_);
        this->pose_=initial_pose_;
    }
    catch(std::exception &e)
    {
        ROS_ERROR("Error due lookup for initial endeffector pose!");
        throw;
    }

    try{
        geometry_msgs::TransformStamped trafo;
        trafo=this->buffer_.lookupTransform(this->planning_frame_,this->base_frame_,ros::Time(0.0),ros::Duration(10.0));
        tf2::fromMsg(trafo.transform,this->planning_offset_);
    }
    catch(std::exception &e)
    {
        ROS_ERROR("Error due lookup for initial endeffector pose!");
        throw;
    }
}
void MoveBase::scope(const ros::TimerEvent &)
{
    geometry_msgs::PoseStamped pose;
    tf2::toMsg(this->pose_,pose.pose);
    pose.header.frame_id=this->base_frame_;
    this->pose_pub_.publish(pose);
}
