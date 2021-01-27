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
tf2_ros::Buffer tfBuffer;

void poseCallback(const geometry_msgs::PoseConstPtr& msg){   
    geometry_msgs::TransformStamped map_base;
    map_base.header.stamp = ros::Time::now();
    map_base.header.frame_id = "/map";
    map_base.child_frame_id = tf::resolve(tf_prefix,"base_footprint");  

    map_base.transform.translation.x = msg->position.x;
    map_base.transform.translation.y = msg->position.y;
    map_base.transform.translation.z = msg->position.z;
    map_base.transform.rotation.x = msg->orientation.x;
    map_base.transform.rotation.y = msg->orientation.y;
    map_base.transform.rotation.z = msg->orientation.z;
    map_base.transform.rotation.w = msg->orientation.w;     
    static tf2_ros::TransformBroadcaster br;
    br.sendTransform(map_base);
   
  
}

int main(int argc, char** argv){
    ros::init(argc, argv, "map_base_broadcaster");

    ros::NodeHandle private_node("~");
    private_node.getParam("tf_prefix", tf_prefix);

        
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("robot_pose", 10, &poseCallback);

    ros::spin();
    return 0;
};