#include <base_trajectory_analytical/planner_base.h>


//Implementation of a generic PlannerBase class########################################################################################
//#################################################################################################################################
PlannerBase::PlannerBase(ros::NodeHandle &nh):nh(nh)
{
    this->frame_name="/map";

    this->tim_sampling=this->nh.createTimer(ros::Duration(0.05),&PlannerBase::plan,this);

    this->pub_current_odometry=nh.advertise<nav_msgs::Odometry>("trajectory_odom",10);
    this->pub_current_vel=nh.advertise<geometry_msgs::Twist>("trajectory_vel",10);
    this->pub_current_pose=nh.advertise<geometry_msgs::PoseStamped>("trajectory_pose",10);
    this->sub_reset_pose=nh.subscribe("reset_pose",10,&PlannerBase::resetCallback,this);

    ros::NodeHandle priv("~");
    this->set_start_service=priv.advertiseService("start_planner",&PlannerBase::srv_start,this);
    this->set_stop_service=priv.advertiseService("stop_planner",&PlannerBase::srv_stop,this);

    this->paused=ros::Duration(0,0);
    this->start_reference.setIdentity();
    this->is_planning=false;
    this->iterations=1;
    this->iterations_counter=0;
    
    this->start_pose=tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(0.0,0.0,0.0));
    this->vel=tf::Vector3(0,0,0);
    this->pos=tf::Vector3(0,0,0);
    this->ang_vel=0.0;
    this->orientation=tf::createQuaternionFromYaw(0);
}
void PlannerBase::resetCallback(geometry_msgs::PoseWithCovarianceStamped msg)
{
    tf::Pose pose;
    tf::poseMsgToTF(msg.pose.pose,pose);
    this->setStartPose(pose);
}


void PlannerBase::plan(const ros::TimerEvent& event)
{
    if(this->iterations_counter==this->iterations)
    {
        stop();
    }
    if(this->is_planning)
    {
        ros::Duration local_time;
        local_time=ros::Time::now()-this->start_time-this->paused;        
        
        this->vel=this->get_velocity(local_time);
        this->pos=this->get_position(local_time);
        this->orientation=this->get_orientation(local_time);
        this->ang_vel=this->get_angular_velocity(local_time);
        this->check_period(local_time);
        this->transformValues(this->start_reference);
        
    }
    this->publish();
    
}


void PlannerBase::start()
{
    this->start_time=ros::Time::now();
    this->is_planning=true;
    this->start_reference=this->getTransform(this->start_pose);
}

void PlannerBase::pause()
{
    this->is_planning=false;
    this->is_paused=true;
}

bool PlannerBase::srv_start(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    this->start();
    return true;
}

void PlannerBase::stop()
{
    ROS_INFO("Shutting down node: %s",ros::this_node::getName().c_str());
    ros::shutdown();
}
bool PlannerBase::srv_stop(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    this->stop();
    return true;
}

void PlannerBase::setStartPose(tf::Pose pose)
{
    this->start_pose=pose;
    this->pos=pose.getOrigin();
    this->orientation=pose.getRotation();
}


tf::Transform PlannerBase::getTransform(tf::Pose pose)
{   
    tf::Pose PlannerBase_start( this->get_orientation(ros::Duration(0)),
                            this->get_position(ros::Duration(0)));

    ROS_INFO(   "PlannerBase starts at: x-%lf y-%lf theta-%lf",
                PlannerBase_start.getOrigin().x(),
                PlannerBase_start.getOrigin().y(),
                tf::getYaw(PlannerBase_start.getRotation()));
    
    tf::Transform trafo(pose*PlannerBase_start.inverse());

    ROS_INFO(   "Trafo applied on PlannerBase values: x-%lf y-%lf z-%lf theta-%lf ",
                trafo.getOrigin().x(),
                trafo.getOrigin().y(),
                trafo.getOrigin().z(),               
                tf::getYaw(trafo.getRotation()));
    return trafo;
}


void PlannerBase::transformValues(tf::Transform trafo)
{
    tf::Transform rotate(trafo.getRotation());
    this->vel=rotate*this->vel;
    this->pos=trafo*this->pos;
    this->orientation=rotate*this->orientation;
}


void PlannerBase::publish()
{    
    nav_msgs::Odometry msg2;
    msg2.header.frame_id=this->frame_name;
    msg2.header.stamp=ros::Time::now();
    tf::Pose pose(this->orientation,this->pos);
    tf::poseTFToMsg(pose,msg2.pose.pose);
    tf::vector3TFToMsg(this->vel,msg2.twist.twist.linear);
    msg2.twist.twist.angular.z=this->ang_vel;    
    this->pub_current_odometry.publish(msg2);

    geometry_msgs::Twist msg_vel;
    msg_vel=msg2.twist.twist;
    this->pub_current_vel.publish(msg_vel);

    geometry_msgs::PoseStamped msg_pose;
    msg_pose.header=msg2.header;
    msg_pose.pose=msg2.pose.pose;
    this->pub_current_pose.publish(msg_pose);

}

void PlannerBase::load()
{
    ros::NodeHandle priv("~");
    if(!priv.getParam(PARAM_ITERATION,this->iterations))
    {
        ROS_WARN("Could not load %s",priv.resolveName(PARAM_ITERATION).c_str());
    }
    this->loadChild();
}






