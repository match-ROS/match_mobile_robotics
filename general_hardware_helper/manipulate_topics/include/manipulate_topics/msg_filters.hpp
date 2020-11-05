#include<manipulate_topics/msg_filter_base.hpp>
#include<manipulate_topics/msg_mean_filter.hpp>
#include<manipulate_topics/msg_diff_filter.hpp>

#include<geometry_msgs/Wrench.h>
#include<geometry_msgs/WrenchStamped.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/TwistStamped.h>
#include<sensor_msgs/JointState.h>

#define ADD_MEAN_FILTER(message_ns,message_type)\
namespace message_filters\
{\
    typedef MessageMeanFilter<message_ns::message_type> MeanFilter##message_type ;\
}

#define ADD_DIFF_FILTER(message_ns,message_type)\
namespace message_filters\
{\
    typedef MessageDiffFilter<message_ns::message_type> DiffFilter##message_type ;\
}



ADD_MEAN_FILTER(geometry_msgs,WrenchStamped)
ADD_MEAN_FILTER(geometry_msgs,Wrench)
ADD_MEAN_FILTER(geometry_msgs,Twist)
ADD_MEAN_FILTER(geometry_msgs,TwistStamped)
ADD_MEAN_FILTER(sensor_msgs,JointState)


ADD_DIFF_FILTER(geometry_msgs,TwistStamped)
