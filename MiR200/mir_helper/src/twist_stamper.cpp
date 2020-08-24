#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>


ros::Publisher chatter_pub;

void callback_pose(geometry_msgs::Twist msg)
{
    geometry_msgs::TwistStamped twist;
    twist.header.frame_id="/map";
    twist.header.stamp=ros::Time::now();
    twist.twist=msg;
    chatter_pub.publish(twist);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "twist_stamper");

  ros::NodeHandle n;

  ros::NodeHandle private_("~");

  chatter_pub = n.advertise<geometry_msgs::TwistStamped>("twist_out", 10);

  ros::Subscriber pos_sub=n.subscribe("twist_in",10,callback_pose);

  
  ros::spin();


  return 0;
}