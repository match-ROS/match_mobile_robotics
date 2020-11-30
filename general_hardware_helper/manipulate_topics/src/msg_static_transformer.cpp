//Directly included within the hpp
template <class T>
MessageStaticTransformer<T>::MessageStaticTransformer(ros::NodeHandle &nh):MessageFilterBase<T>(nh)
{  
    this->buffer_=std::make_unique<boost::circular_buffer<T>>(1);
    tf2_ros::Buffer tfBuffer;
    tfBuffer.setUsingDedicatedThread(true);
    tf2_ros::TransformListener tfListener(tfBuffer);


    ros::NodeHandle priv("~");

    if(priv.getParam("source_frame",this->source_frame_) && priv.getParam("target_frame",this->target_frame_))
    {
        try
        {
            this->transform_ = tfBuffer.lookupTransform(this->target_frame_,this->source_frame_,
                                    ros::Time(0),
                                    ros::Duration(5));
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            return;
        }
    }
    else
    {
        ROS_ERROR("Could not find trasnformation!");
        return;
    }

   
    
}
template <class T>
T MessageStaticTransformer<T>::filter()
{
    if(this->buffer_->size()<1) {return T();}
    using namespace msg_operators;

    T ret;    
    tf2::doTransform(this->buffer_->at(0),ret,this->transform_);
    return ret;
}