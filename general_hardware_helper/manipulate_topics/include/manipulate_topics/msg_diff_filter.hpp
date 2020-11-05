#include <ros/ros.h>

template<class T>
class MessageDiffFilter: public MessageFilterBase<T>{
    public:
        MessageDiffFilter(ros::NodeHandle &nh);
        
    private:
        T filter() override; 
};
#include "../src/msg_diff_filter.cpp"