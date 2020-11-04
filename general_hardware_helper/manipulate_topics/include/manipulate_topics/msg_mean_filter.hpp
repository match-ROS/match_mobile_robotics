#pragma once
#include <manipulate_topics/msg_filter_base.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <numeric>
#include <manipulate_topics/msg_operators.hpp>
#include <dynamic_reconfigure/server.h>
#include<manipulate_topics/MeanConfig.h>

/**
 * @brief Class that implements a mean filter for a given Topic type.
 * 
 * |Ros-Parameter | Desciption|
 * |---- | -----|
 * |~sample_number | Number of samples the mean is calculated from|
 * 
 * @tparam T Type of the topic to be filtered. Operators + and / must be implemented within the msg_operators namespace.
 */

template<class T>
class MessageMeanFilter: public MessageFilterBase<T>{
    public:
        typedef boost::accumulators::accumulator_set<T,
                                                    boost::accumulators::stats<boost::accumulators::tag::mean>>
                                                    accumulator;
        
        MessageMeanFilter(ros::NodeHandle &nh);
        
    private:
        dynamic_reconfigure::Server<manipulate_topics::MeanConfig> server_; 
        void dynConfigcallback(manipulate_topics::MeanConfig &config, uint32_t level); ///<Callback for the dynamic reconfigure server
        int samples_num_;
        T filter() override; 
};
#include "../src/msg_mean_filter.cpp"