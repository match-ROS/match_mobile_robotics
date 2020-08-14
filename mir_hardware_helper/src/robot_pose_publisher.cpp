#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>


ros::Publisher chatter_pub;

void callback_pose(geometry_msgs::Pose msg)
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id="/map";
    pose.header.stamp=ros::Time::now();
    pose.pose=msg;
    chatter_pub.publish(pose);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::NodeHandle private_("~");

  chatter_pub = n.advertise<geometry_msgs::PoseStamped>("robot_pose_stamped", 10);

  ros::Subscriber pos_sub=n.subscribe("robot_pose",10,callback_pose);

  
  ros::spin();


  return 0;
}