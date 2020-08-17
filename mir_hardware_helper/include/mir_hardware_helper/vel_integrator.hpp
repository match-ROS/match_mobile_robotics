#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<mir_hardware_helper/SetInitialPose.h>
#include<tf/tf.h>


class VelIntegrator{
    public:
    VelIntegrator(ros::NodeHandle &nh,std::string topic_name_vel,std::string topic_name_pose):nh_(nh)
    {
        this->sub_=this->nh_.subscribe(topic_name_vel,10,&VelIntegrator::callbackIntegrate,this);
        this->pub_=this->nh_.advertise<geometry_msgs::PoseStamped>(topic_name_pose,10);
        this->initial_server_=this->nh_.advertiseService("set_initial_pose",&VelIntegrator::callbackSetInitial,this);
        this->pose_.setIdentity();
    }

    private:
    void callbackIntegrate(geometry_msgs::Twist msg)
    {
        ros::Duration d_t;
        ros::Time now=ros::Time::now();
        d_t=now-this->time_;       
        this->time_=now;

        tf::Vector3 lin;
        tf::vector3MsgToTF(msg.linear,lin);
        lin.setZ(0.0);
       
        tf::Transform rot;
        rot.setRotation(this->pose_.getRotation());
        lin=rot*lin;
        
        tf::Vector3 ang;
        tf::vector3MsgToTF(msg.angular,ang);

        this->pose_.setOrigin(this->pose_.getOrigin()+lin*d_t.toSec());
        this->pose_.setRotation(tf::createQuaternionFromYaw(ang.z()*d_t.toSec())*this->pose_.getRotation());

        geometry_msgs::PoseStamped pose_msg;
        tf::poseTFToMsg(this->pose_,pose_msg.pose);
        pose_msg.header.stamp=now;
        this->pub_.publish(pose_msg);         
    }
    bool callbackSetInitial(mir_hardware_helper::SetInitialPoseRequest &req,mir_hardware_helper::SetInitialPoseResponse &res)
    {
        tf::poseMsgToTF(req.pose.pose,this->pose_);
        res.succeed=true;
        this->time_=ros::Time::now();
    }
    tf::Pose pose_;
    ros::Time time_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::ServiceServer initial_server_;

};