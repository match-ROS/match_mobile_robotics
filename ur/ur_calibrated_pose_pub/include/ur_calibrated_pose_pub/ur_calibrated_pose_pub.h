#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <ur_calibrated_pose_pub/utils/ur_calibration_consumer.h>
namespace ur_calibrated_pose_pub
{

class URCalibratedPosePub
{
public:
  URCalibratedPosePub(ros::NodeHandle nh, ros::NodeHandle private_nh);

  void init();
  void execute();

private:
  // --- internal methods ---
  void readParams();
  //void getCalibratedDHParameter();   // vorhanden aus deiner bisherigen Version
  bool computePose(Eigen::Isometry3d& out_T_base_tcp);
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

  // --- ROS ---
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber joint_state_subscriber_;
  ros::Publisher ur_calibrated_pose_publisher_;

  // --- parameters ---
  std::string ur_joint_state_topic_name_;
  std::string joint_prefix_;
  std::string dh_parameter_switch_;

  // --- DH data ---
  std::vector<dh_utils::DHTransformation> calibrated_dh_transformations_list_;
  std::vector<dh_utils::DHTransformation> ideal_dh_transformations_list_;
};

}  // namespace ur_calibrated_pose_pub
