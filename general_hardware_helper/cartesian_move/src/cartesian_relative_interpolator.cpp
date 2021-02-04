#include <cartesian_move/cartesian_relative_interpolator.h>

CartesianRelativeInterpolator::CartesianRelativeInterpolator(ros::NodeHandle &nh):CartesianInterpolator(nh)  

{}


std::vector<geometry_msgs::PoseStamped> CartesianRelativeInterpolator::calcPath(geometry_msgs::PoseStamped source,geometry_msgs::PoseStamped target)
{
    tf2::Transform source_tf;
    tf2::Transform target_tf;
    tf2::fromMsg(source.pose,source_tf);
    tf2::fromMsg(target.pose,target_tf);
  

    std::vector<geometry_msgs::PoseStamped> path=this->interpolate(target_tf);
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