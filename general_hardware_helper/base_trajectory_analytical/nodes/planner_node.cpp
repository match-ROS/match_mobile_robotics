#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <base_trajectory_analytical/planner_base.h>
#include <base_trajectory_analytical/spiral_planner.h>
#include <base_trajectory_analytical/lissajous_planner.h>
#include <base_trajectory_analytical/primitive_planner.h>

enum ControllerTypes{
    lissajous=0,
    spiral=1,
    primitive=2
};


int main(int argc,char**argv)
{
    //Choose planner
    ros::init(argc,argv,"planner_node");
  
    ros::NodeHandle nh;
    PlannerBase* planner;
    
    int type; 
    ros::NodeHandle priv("~");   
    if(!priv.getParam("type",type))
    {
        ROS_ERROR("Could not find parameter 'type' for planner");
        return 0;

    }
    bool pause;
    if(!priv.getParam("pause",pause))
    {
        pause=true;
    }
   
    switch(type)
    {
        case lissajous:
        {
            ROS_INFO("Starting LissajousPlanner!"); 
            planner=new LissajousPlanner(nh);
            break;
        }
        case spiral:
        {
            ROS_INFO("Starting Spiral Planner");
            planner=new Spiralplanner(nh);
        }
        case primitive:
        {
            ROS_INFO("Starting Primitive Planner");
            planner=new PrimitivePlanner(nh);
        }
        default:
        {
            return 0;
        }
              

    }


    tf::Pose reference(tf::createIdentityQuaternion(),tf::Vector3(0,0,0));
    geometry_msgs::PoseWithCovarianceStamped pose;
    ROS_INFO_STREAM("Waiting for start_pose");
    pose=*ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("start_pose");
    geometry_msgs::Pose tmp= pose.pose.pose;
    ROS_INFO_STREAM("Got start_pose");

    tf::poseMsgToTF(tmp,reference);

    planner->setStartPose(reference);
    planner->load();
    
    ros::Duration(2).sleep();
    if(!pause)
    {
        planner->start();
    }    
    ros::spin();
    delete planner;
}