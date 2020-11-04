#include<manipulate_topics/msg_mean_filter.hpp>
#include<manipulate_topics/msg_filter_base.hpp>

#include<geometry_msgs/Wrench.h>
#include<geometry_msgs/WrenchStamped.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/TwistStamped.h>
#include<sensor_msgs/JointState.h>


namespace message_filters
{
    typedef MessageMeanFilter<geometry_msgs::Wrench> MeanFilterWrench;
    typedef MessageMeanFilter<geometry_msgs::WrenchStamped> MeanFilterWrenchStamped;   
    typedef MessageMeanFilter<geometry_msgs::Twist> MeanFilterTwist;
    typedef MessageMeanFilter<geometry_msgs::TwistStamped> MeanFilterTwistStamped;
    typedef MessageMeanFilter<sensor_msgs::JointState> MeanFilterJointState;
    
}
