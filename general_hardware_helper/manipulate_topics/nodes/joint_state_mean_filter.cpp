#include<manipulate_topics/msg_filters.hpp>
#include<sensor_msgs/JointState.h>



int main(int argc,char** argv)
{
    ros::init(argc,argv,"mean_filter");
    ros::NodeHandle nh;
    MessageMeanFilter<sensor_msgs::JointState> filter(nh);
    ros::spin();
}