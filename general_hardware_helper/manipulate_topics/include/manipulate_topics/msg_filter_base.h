#pragma once
#include <boost/accumulators/accumulators.hpp>
#include <boost/circular_buffer.hpp>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>

template<class T>
class MessageFilterBase{
    public:
        template<typename Features, typename Weight=void>
        using accumulator=boost::accumulators::accumulator_set<T,Features,Weight>;
        MessageFilterBase(ros::NodeHandle &nh);

    protected:
        virtual T filter()=0;
        std::unique_ptr<boost:: circular_buffer<T>> buffer_;
    private:   
        void inputCallback(T msg);     
        ros::NodeHandle nh_;
        ros::Subscriber input_;
        ros::Publisher output_; 
};
template class MessageFilterBase<geometry_msgs::WrenchStamped>;
template class MessageFilterBase<sensor_msgs::JointState>;