#include<manipulate_topics/msg_filters.hpp>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"mean_filter");
    ros::NodeHandle nh;
    message_filters::DiffFilterTwistStamped filter(nh);
    ros::spin();
}