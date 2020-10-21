#include<manipulate_topics/msg_filters.hpp>
#include<geometry_msgs/TwistStamped.h>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"mean_filter");
    ros::NodeHandle nh;
    MessageMeanFilter<geometry_msgs::TwistStamped> filter(nh);
    ros::spin();
}