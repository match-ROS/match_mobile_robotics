#include<cartesian_move/relative_move.h>

RelativeMove::RelativeMove(ros::NodeHandle &nh):MoveBase(nh)
{

}

void RelativeMove::init()
{
    MoveBase::init();
    this->planning_rotation_=tf2::Transform(this->planning_offset_.getRotation(),tf2::Vector3(0.0,0.0,0.0));
    this->relative_subscriber_=this->nh_.subscribe("pose_in_msg",10,&RelativeMove::inputMsgCallback,this);
    this->relative_service_=this->nh_.advertiseService("pose_in_srv",&RelativeMove::inputSrvCallback,this);
}

void RelativeMove::inputMsgCallback(geometry_msgs::PoseStamped msg)
{
    tf2::Transform trafo_msg;   
    tf2::fromMsg(msg.pose,trafo_msg);
    this->pose_=tf2::Transform( this->initial_pose_.getRotation()*trafo_msg.getRotation(),
                                this->planning_rotation_.inverse()*trafo_msg.getOrigin()+this->initial_pose_.getOrigin()); 
    
}
bool RelativeMove::inputSrvCallback(cartesian_move::SetPoseRequest &req,cartesian_move::SetPoseResponse &res)
{
    tf2::Transform trafo_msg;  
    tf2::fromMsg(req.pose.pose,trafo_msg);
    this->pose_=tf2::Transform( this->pose_.getRotation()*trafo_msg.getRotation(),
                                this->planning_rotation_.inverse()*trafo_msg.getOrigin()+this->pose_.getOrigin());
    res.executed=true;
    return true;
    
}