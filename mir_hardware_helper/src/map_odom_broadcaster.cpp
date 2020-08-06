#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>



std::string tf_prefix="";
geometry_msgs::Transform map_base;
tf2_ros::Buffer tfBuffer;

void poseCallback(const geometry_msgs::PoseConstPtr& msg){   
    geometry_msgs::Transform map_base;
    map_base.translation.x = msg->position.x;
    map_base.translation.y = msg->position.y;
    map_base.translation.z = msg->position.z;
    map_base.rotation.x = msg->orientation.x;
    map_base.rotation.y = msg->orientation.y;
    map_base.rotation.z = msg->orientation.z;
    map_base.rotation.w = msg->orientation.w;

     
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped odom_base;   
    tf2_ros::TransformListener tfListener(tfBuffer);

    try{
        odom_base = tfBuffer.lookupTransform(   
                                                tf::resolve(tf_prefix,"odom"),
                                                tf::resolve(tf_prefix,"base_footprint"),
                                                ros::Time(0));
        
        

        //alloc tf2 objects from the msg and lookup
        tf2::Transform odom_base_transform;
        tf2::fromMsg(odom_base.transform,odom_base_transform);
        tf2::Transform map_base_transform;
        tf2::fromMsg(map_base,map_base_transform);

        //build new map odom trafo
        geometry_msgs::TransformStamped map_odom;
        map_odom.header.stamp = ros::Time::now();
        map_odom.header.frame_id = "/map";
        map_odom.child_frame_id = tf::resolve(tf_prefix,"odom");  
        map_odom.transform=tf2::toMsg(map_base_transform*odom_base_transform.inverse());       
        br.sendTransform(map_odom);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
  
}

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf2_broadcaster");

    ros::NodeHandle private_node("~");
    private_node.getParam("tf_prefix", tf_prefix);

        
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("robot_pose", 10, &poseCallback);

    ros::spin();
    return 0;
};