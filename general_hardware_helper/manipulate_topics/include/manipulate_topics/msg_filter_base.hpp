#pragma once
#include <boost/accumulators/accumulators.hpp>
#include <boost/circular_buffer.hpp>
#include <ros/ros.h>



/**
 * @brief  Base class for topic filters (convulutional filters)
 * 
 * |Subscribed Topic|Description|
 * |-----|-----|
 * |input|The topic the value to be filtered is published at|
 * |output|The topic the filtered value is published at|
 * 
 * 
 * @tparam T Tpye of the Message to be filtered. Operators for the derived filteres must be implemented.
 */
template<class T>
class MessageFilterBase{
    public:
        /**
         * @brief Construct a new Message Filter Base object
         * 
         * @param nh Nodehandle for handling ros namespaces
         * 
         */
        MessageFilterBase(ros::NodeHandle &nh);

    protected:
        /**
         * @brief Procedure the filter algorithm is implemented in.
         * 
         * Uses values from the buffer_ and filters them.
         * 
         * @return T Filtered value.
         */
        virtual T filter()=0;

        std::unique_ptr<boost:: circular_buffer<T>> buffer_;    //<Circular buffer to hold the specified number of samples
    private:   
        void inputCallback(T msg);     
        ros::NodeHandle nh_;
        ros::Subscriber input_;
        ros::Publisher output_; 
};
#include "../src/msg_filter_base.cpp"