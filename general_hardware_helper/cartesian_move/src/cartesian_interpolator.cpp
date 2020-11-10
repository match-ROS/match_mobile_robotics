#include <cartesian_move/cartesian_interpolator.h>

CartesianInterpolator::CartesianInterpolator(ros::NodeHandle &nh):  nh_(nh),
                                                                    as_(nh_, 
                                                                        "cartesian_interpolator",
                                                                         boost::bind(&CartesianInterpolator::executeCallback, this, _1),
                                                                         false),
                                                                    listener_(buffer_)

{
    this->buffer_.setUsingDedicatedThread(true);
    ros::NodeHandle param("~");
    if(!param.getParam("max_velocity",this->max_vel_))
    {
        ROS_ERROR_STREAM("Interpolator could not find parameter for max_velocity");
    }
    if(!param.getParam("time_step",this->time_step_))
    {
        ROS_ERROR_STREAM("Interpolator could not find parameter for time_step");
    }
    if(!param.getParam("ee_frame",this->ee_frame_))
    {
        ROS_ERROR_STREAM("Interpolator could not find parameter for ee_frame");
    }    
    if(!param.getParam("base_frame",this->base_frame_))
    {
        ROS_ERROR_STREAM("Interpolator could not find parameter for base_frame");
    }    
    this->pose_publisher_=this->nh_.advertise<geometry_msgs::PoseStamped>("pose_interpolated",5);
    this->as_.start();
}

void CartesianInterpolator::executeCallback(const cartesian_move::CartesianInterpolationGoalConstPtr &goal)
{
    geometry_msgs::PoseStamped pose;
    try{
        geometry_msgs::TransformStamped trafo;
        trafo=this->buffer_.lookupTransform(this->base_frame_,this->ee_frame_,ros::Time(0.0),ros::Duration(10.0));
        pose.pose.position.x=trafo.transform.translation.x;
        pose.pose.position.y=trafo.transform.translation.y;
        pose.pose.position.z=trafo.transform.translation.z;
        pose.pose.orientation.x=trafo.transform.rotation.x;
        pose.pose.orientation.y=trafo.transform.rotation.y;
        pose.pose.orientation.z=trafo.transform.rotation.z;
        pose.pose.orientation.w=trafo.transform.rotation.w;
    }
    catch(std::exception &e)
    {
        ROS_ERROR("Error due lookup for initial endeffector pose!");
        throw;
        return;
    }


    ros::Time start_time=ros::Time::now();
    std::vector<geometry_msgs::PoseStamped> path=this->calcPath(pose, goal->target);
    ros::Duration sleep_time(this->time_step_);
    
    for(int i=0;i<path.size();i++)
    {
        cartesian_move::CartesianInterpolationFeedback feed;
        feed.time_left=this->time_-i*time_step_;
        this->as_.publishFeedback(feed);
        path.at(i).header.stamp=start_time;
        this->pose_publisher_.publish(path.at(i));
        start_time+=sleep_time;
        sleep_time.sleep();
    }
}

std::vector<geometry_msgs::PoseStamped> CartesianInterpolator::interpolate(tf2::Transform diff)
{
    double dx=diff.getOrigin().x();
    double dy=diff.getOrigin().y();
    double dz=diff.getOrigin().z();

    std::vector<double> values {dx,dy,dz};
    double max=*std::max_element(values.begin(),values.end());
    this->time_=max/this->max_vel_;

    int steps=ceil(this->time_/this->time_step_);
    
    std::vector<double> vels;
    for (size_t i = 0; i < values.size(); i++)
    {
        vels.push_back(values.at(i)/this->time_);
    }

    std::vector<geometry_msgs::PoseStamped> path;
    path.push_back(geometry_msgs::PoseStamped());
    for(int i=0;i<steps;i++)
    {
        geometry_msgs::PoseStamped pose_old=path.back();
        geometry_msgs::PoseStamped pose_new;  

        pose_new.pose.position.x=vels.at(0)*this->time_step_+pose_old.pose.position.x;
        pose_new.pose.position.y=vels.at(1)*this->time_step_+pose_old.pose.position.y;
        pose_new.pose.position.z=vels.at(2)*this->time_step_+pose_old.pose.position.z;
        pose_new.pose.orientation=pose_old.pose.orientation;
        pose_new.header.frame_id=this->base_frame_;
        path.push_back(pose_new);
    }

    return path;
}

std::vector<geometry_msgs::PoseStamped> CartesianInterpolator::calcPath(geometry_msgs::PoseStamped source,geometry_msgs::PoseStamped target)
{
    tf2::Transform source_tf;
    tf2::Transform target_tf;
    tf2::fromMsg(source.pose,source_tf);
    tf2::fromMsg(target.pose,target_tf);
    tf2::Transform diff;
    diff=tf2::Transform(tf2::Quaternion::getIdentity(),target_tf.getOrigin()-source_tf.getOrigin());

    std::vector<geometry_msgs::PoseStamped> path=this->interpolate(diff);
    std::vector<geometry_msgs::PoseStamped> path_off;
    for(auto point:path)
    {
        ROS_INFO_STREAM("IN");
        geometry_msgs::PoseStamped pose_off;
        pose_off.header.frame_id=this->base_frame_;
        pose_off.pose.position.x=point.pose.position.x+source.pose.position.x;
        pose_off.pose.position.y=point.pose.position.y+source.pose.position.y;
        pose_off.pose.position.z=point.pose.position.z+source.pose.position.z;
        pose_off.pose.orientation=point.pose.orientation;
        path_off.push_back(pose_off);
    }
    return path_off;
}