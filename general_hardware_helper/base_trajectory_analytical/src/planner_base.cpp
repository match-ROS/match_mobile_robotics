#include <base_trajectory_analytical/planner_base.h>


//Implementation of a generic PlannerBase class########################################################################################
//#################################################################################################################################
PlannerBase::PlannerBase(ros::NodeHandle &nh):nh(nh)
{
    this->frame_name="/map";

    this->tim_sampling=this->nh.createTimer(ros::Duration(0.01),&PlannerBase::plan,this);

    this->pub_current_odometry=nh.advertise<nav_msgs::Odometry>("trajectory_odom",10);
    this->pub_current_vel=nh.advertise<geometry_msgs::Twist>("trajectory_vel",10);
    this->pub_current_pose=nh.advertise<geometry_msgs::PoseStamped>("trajectory_pose",10);
    this->pub_path=nh.advertise<nav_msgs::Path>("simulated_path",10);
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

void PlannerBase::load()
{
    ros::NodeHandle priv("~");
    if(!priv.getParam(PARAM_ITERATION,this->iterations))
    {
        ROS_WARN("Could not load %s",priv.resolveName(PARAM_ITERATION).c_str());
    }
    this->loadChild();
    this->simulate();
}



void PlannerBase::plan(const ros::TimerEvent& event)
{
    if(this->iterations_counter==this->iterations)
    {
        this->iterations_counter=0;
        this->pause();
    }
    if(this->is_planning)
    {
        ros::Duration local_time;
        local_time=ros::Time::now()-this->start_time-this->paused;      
        this->calc_values(local_time); 
        
    }
    this->publish();
    
}

void PlannerBase::calc_values(ros::Duration local_time)
{   
    this->pos=this->get_position(local_time);
    this->vel=this->get_velocity(local_time);
    this->orientation=this->get_orientation(local_time);
    this->ang_vel=this->get_angular_velocity(local_time);
    this->check_period(local_time);
    this->transformValues(this->start_reference,this->pos,this->vel,this->orientation);
}



void PlannerBase::resetCallback(geometry_msgs::PoseWithCovarianceStamped msg)
{
    tf::Pose pose;
    tf::poseMsgToTF(msg.pose.pose,pose);
    this->setStartPose(pose);
}


void PlannerBase::start()
{
    ROS_INFO_STREAM("Starting planner");
    this->start_time=ros::Time::now();
    this->is_planning=true;
}

void PlannerBase::pause()
{
    ROS_INFO_STREAM("Pausing planner");
    this->is_planning=false;
}

bool PlannerBase::srv_start(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    this->start();
    return true;
}

bool PlannerBase::srv_stop(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    this->pause();
    return true;
}

void PlannerBase::setStartPose(tf::Pose pose)
{
    this->start_pose=pose;
    this->start_reference=this->getTransform(pose);
    this->calc_values(ros::Duration(0.0));
    this->vel=tf::Vector3(0.0,0.0,0.0);
    this->ang_vel=0.0;
    this->simulate();
}


void PlannerBase::publish()
{    
    nav_msgs::Odometry msg_odom;
    msg_odom.header.frame_id=this->frame_name;
    msg_odom.header.stamp=ros::Time::now();
    tf::Pose pose(this->orientation,this->pos);
    tf::poseTFToMsg(pose,msg_odom.pose.pose);
    tf::vector3TFToMsg(this->vel,msg_odom.twist.twist.linear);
    msg_odom.twist.twist.angular.z=this->ang_vel;    
    this->pub_current_odometry.publish(msg_odom);

    geometry_msgs::Twist msg_vel;
    msg_vel=msg_odom.twist.twist;
    this->pub_current_vel.publish(msg_vel);

    geometry_msgs::PoseStamped msg_pose;
    msg_pose.header=msg_odom.header;
    msg_pose.pose=msg_odom.pose.pose;
    this->pub_current_pose.publish(msg_pose);

    this->pub_path.publish(this->trajectory_);

}


void PlannerBase::simulate()
{
    ROS_INFO_STREAM("Starting path simulation");
    this->trajectory_=nav_msgs::Path();
    ros::Duration sim_time (0.0);
    while(sim_time.toSec()<300.0 && this->iterations_counter<1)
    {
        tf::Vector3 pos=this->get_position(sim_time);
        tf::Vector3 vel=this->get_velocity(sim_time);
        tf::Quaternion ori=this->get_orientation(sim_time);
        this->check_period(sim_time);
        this->transformValues(this->start_reference,pos,vel,ori);

        tf::Pose pos_tf=tf::Pose(ori,pos);
     
        geometry_msgs::PoseStamped pose;
        tf::poseTFToMsg(pos_tf,pose.pose);
        pose.header.frame_id="/map";
        this->trajectory_.poses.push_back(pose);
        
        sim_time+=ros::Duration(1.0);  
             
    }
    this->trajectory_.header.frame_id="/map";
    this->iterations_counter=0;
    ROS_INFO_STREAM("Path simulation done");
}

tf::Transform PlannerBase::getTransform(tf::Pose pose)
{   
    tf::Pose planner_start( this->get_orientation(ros::Duration(0)),
                            this->get_position(ros::Duration(0)));

    
    tf::Transform trafo=pose*planner_start.inverse();
    trafo.setRotation(pose.getRotation()*planner_start.inverse().getRotation());
    
    ROS_INFO(   "Trafo applied on PlannerBase values: x-%lf y-%lf z-%lf theta-%lf ",
                trafo.getOrigin().x(),
                trafo.getOrigin().y(),
                trafo.getOrigin().z(),               
                tf::getYaw(trafo.getRotation()));
    return trafo;
}


void PlannerBase::transformValues(tf::Transform trafo,tf::Vector3 &pos,tf::Vector3 &vel,tf::Quaternion &ori)
{
    tf::Transform rotate(trafo.getRotation(),tf::Vector3(0.0,0.0,0.0));   
    pos=trafo*pos;
    vel=rotate*vel;
    ori=rotate*ori;
}

