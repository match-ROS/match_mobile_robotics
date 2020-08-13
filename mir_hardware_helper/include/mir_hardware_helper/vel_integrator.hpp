#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<tf/tf.h>

class VelIntegrator{
    public:
    VelIntegrator(ros::NodeHandle &nh,std::string topic_name_vel,std::string topic_name_pose):nh_(nh)
    {
        this->sub_=this->nh_.subscribe(topic_name_vel,10,&VelIntegrator::callbackIntegrate,this);
        this->pub_=this->nh_.advertise<geometry_msgs::PoseStamped>(topic_name_pose,10);
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
        tf::Vector3 ang;
        tf::vector3MsgToTF(msg.linear,ang);

        tf::Pose d_pose;
        d_pose=tf::Pose(tf::createQuaternionFromYaw(ang.z()*d_t.toSec()),
                        lin*d_t.toSec());

       
        this->pose_=d_pose*this->pose_;

        geometry_msgs::PoseStamped pose_msg;
        tf::poseTFToMsg(this->pose_,pose_msg.pose);
        pose_msg.header.stamp=now;
        this->pub_.publish(pose_msg); 
        
    }
    tf::Pose pose_;
    ros::Time time_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

};