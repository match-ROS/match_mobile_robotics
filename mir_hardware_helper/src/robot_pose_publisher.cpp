#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::PoseStamped pose;

void callback_pose(geometry_msgs::Pose msg)
{
    pose.header.frame_id="/map";
    pose.header.stamp=ros::Time::now();
    pose.pose=msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::NodeHandle private_("~");

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("robot_pose_stamped", 10);

  ros::Subscriber pos_sub=n.subscribe("robot_pose",10,callback_pose);

  double hz=0.0;
    ros::Rate loop_rate(10.0);
  if(private_.getParam("rate",hz))
  {
    loop_rate=ros::Rate(hz);
  }

  
  

  while (ros::ok())
  {
    chatter_pub.publish(pose);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}