#include<ros/ros.h>
#include<geometry_msgs/TwistStamped.h>
#include<mir_hardware_helper/SetInitialPose.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<std_srvs/Empty.h>
#include<tf/tf.h>


class VelIntegrator{
    public:
    VelIntegrator(ros::NodeHandle &nh,std::string topic_name_vel,std::string topic_name_pose):nh_(nh)
    {
        this->sub_=this->nh_.subscribe(topic_name_vel,10,&VelIntegrator::callbackIntegrate,this);
        this->pose_initializer_=this->nh_.subscribe("initialpose",10,&VelIntegrator::callbackSetInitial,this);
        this->pub_=this->nh_.advertise<geometry_msgs::PoseStamped>(topic_name_pose,10);
        ros::NodeHandle priv("~");
        this->initial_server_=priv.advertiseService("reset",&VelIntegrator::callbackReset,this);
        this->pose_.setIdentity();
        
        priv.getParam("topic_name",this->pose_topic_);
        priv.getParam("frame_id",this->frame_id_);
    }

    private:
    void callbackSetInitial(geometry_msgs::PoseWithCovarianceStamped msg)
    {
        tf::poseMsgToTF(msg.pose.pose,this->pose_);
    }
    void callbackIntegrate(geometry_msgs::TwistStamped msg)
    {
        ros::Duration d_t;
        ros::Time now=ros::Time::now();
        d_t=now-this->time_;       
        this->time_=now;

        tf::Vector3 lin;
        tf::vector3MsgToTF(msg.twist.linear,lin);
        lin.setZ(0.0);
       
        tf::Transform rot;
        rot.setRotation(this->pose_.getRotation());
        lin=rot*lin;
        
        tf::Vector3 ang;
        tf::vector3MsgToTF(msg.twist.angular,ang);

        this->pose_.setOrigin(this->pose_.getOrigin()+lin*d_t.toSec());
        this->pose_.setRotation(tf::createQuaternionFromYaw(ang.z()*d_t.toSec())*this->pose_.getRotation());

        geometry_msgs::PoseStamped pose_msg;
        tf::poseTFToMsg(this->pose_,pose_msg.pose);
        pose_msg.header.stamp=now;
        pose_msg.header.frame_id=this->frame_id_;
        this->pub_.publish(pose_msg);         
    }
    bool callbackReset(std_srvs::EmptyRequest &req,std_srvs::EmptyResponse  &res)
    {
        tf::poseMsgToTF(ros::topic::waitForMessage<geometry_msgs::PoseStamped>(this->pose_topic_,this->nh_)->pose,this->pose_);
        return true;
    }
    
    std::string frame_id_;
    tf::Pose pose_;    
    ros::Time time_;
    std::string pose_topic_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Subscriber pose_initializer_;
    ros::Publisher pub_;
    ros::ServiceServer initial_pose_server_;
    ros::ServiceServer initial_server_;

};