#pragma once
#include <manipulate_topics/msg_filter_base.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <numeric>
#include <manipulate_topics/msg_operators.hpp>
#include<geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
template<class T>
class MessageMeanFilter: public MessageFilterBase<T>{
    public:
        typedef boost::accumulators::accumulator_set<T,
                                                    boost::accumulators::stats<boost::accumulators::tag::mean>>
                                                    accumulator;
        
        MessageMeanFilter(ros::NodeHandle &nh);
        
    private:
        int samples_num_;
        T filter() override; 
};
template class MessageMeanFilter<geometry_msgs::WrenchStamped>;
template class MessageMeanFilter<sensor_msgs::JointState>;