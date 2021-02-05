#pragma once
#include<tf2_ros/buffer.h>
#include<tf2_ros/transform_listener.h>
#include<geometry_msgs/TransformStamped.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>

template<class T>
class MessageStaticTransformer: public MessageFilterBase<T>{
    public:
        MessageStaticTransformer(ros::NodeHandle &nh);
        
    private:
        std::string source_frame_;
        std::string target_frame_;
        geometry_msgs::TransformStamped transform_;
        T filter() override; 
        
};
#include "../src/msg_static_transformer.cpp"