#include<manipulate_topics/msg_filters.hpp>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"static_wrench_transformer");
    ros::NodeHandle nh;
    message_filters::MessageStaticTransformerWrenchStamped transformer(nh);
    ros::spin();
}