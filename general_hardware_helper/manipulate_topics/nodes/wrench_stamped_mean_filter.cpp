#include<manipulate_topics/msg_filters.hpp>
#include<geometry_msgs/WrenchStamped.h>



int main(int argc,char** argv)
{
    ros::init(argc,argv,"mean_filter");
    ros::NodeHandle nh;
    MessageMeanFilter<geometry_msgs::WrenchStamped> filter(nh);
    ros::spin();
}