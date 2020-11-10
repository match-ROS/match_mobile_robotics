//Directly included within the hpp
template <class T>
MessageDiffFilter<T>::MessageDiffFilter(ros::NodeHandle &nh):MessageFilterBase<T>(nh)
{  
    this->buffer_=std::make_unique<boost::circular_buffer<T>>(2);
}
template <class T>
T MessageDiffFilter<T>::filter()
{
    if(this->buffer_->size()<2) {return T();}
    using namespace msg_operators;

    T ret=this->buffer_->at(1)-this->buffer_->at(0);   
    ret.header.stamp=ros::Time::now();
    ret=ret/(this->buffer_->at(1).header.stamp-this->buffer_->at(0).header.stamp).toSec();
    ROS_INFO_STREAM(this->buffer_->at(1).header.stamp.toSec()<<"\t"<<this->buffer_->at(0).header.stamp.toSec()<<"\t"<<(this->buffer_->at(1).header.stamp-this->buffer_->at(0).header.stamp).toSec());
    return ret;
}