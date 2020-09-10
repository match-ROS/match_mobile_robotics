#include <ros/ros.h>
#include <tf/tf.h>

#include <tf2_ros/static_transform_broadcaster.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf2_broadcaster");
    ros::NodeHandle private_node("~");
    ros::NodeHandle nh;
    std::string tf_prefix="";
    private_node.getParam("tf_prefix", tf_prefix);
        
    static tf2_ros::StaticTransformBroadcaster br;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    try{
        geometry_msgs::PoseStamped pose;
        pose=*ros::topic::waitForMessage<geometry_msgs::PoseStamped>("robot_pose_stamped",nh);

        geometry_msgs::Transform map_base;
        map_base.rotation=pose.pose.orientation;
        map_base.translation.x=pose.pose.position.x;
        map_base.translation.y=pose.pose.position.y;
        map_base.translation.z=pose.pose.position.z;


        geometry_msgs::TransformStamped odom_base;   
        odom_base = tfBuffer.lookupTransform(   
                                                tf::resolve(tf_prefix,"odom"),
                                                tf::resolve(tf_prefix,"base_footprint"),
                                                ros::Time(0),ros::Duration(100.0));       
        

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
    ros::spin();
    return 0;
};