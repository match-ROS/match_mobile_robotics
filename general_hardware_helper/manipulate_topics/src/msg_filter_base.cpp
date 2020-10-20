#include <manipulate_topics/msg_filter_base.h>


template<class T>
MessageFilterBase<T>::MessageFilterBase(ros::NodeHandle &nh):
                                    nh_(nh),
                                    input_(this->nh_.subscribe(  "input",
                                                                10,
                                                                &MessageFilterBase<T>::inputCallback,this)),
                                    output_(this->nh_.advertise<T>("output",10))                             
                                    
{        
}

template<class T>
void MessageFilterBase<T>::inputCallback(T msg)
{
    if(!this->buffer_)
    {
        ROS_ERROR_STREAM("Buffer is not allocated!");
        return;
    }
    this->buffer_->push_back(msg);
    this->output_.publish(this->filter());
}
