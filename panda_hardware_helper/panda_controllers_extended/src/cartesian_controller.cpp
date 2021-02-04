// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <panda_controllers_extended/cartesian_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace panda_controllers_extended {

bool CartesianController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianController: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianController: Exception getting state handle: " << e.what());
    return false;
  }

  this->target_pose_sub_=node_handle.subscribe("target_pose",1,&CartesianController::targetPoseCallback,this);
  this->position_d_target_.setZero();
  this->orientation_d_target_.setIdentity();
  return true;
}

void CartesianController::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(initial_pose_.data()));
  
  this->position_d_target_=transform.translation();
  this->orientation_d_target_ =transform.linear();
}

void CartesianController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) 
{
  Eigen::Transform<double,3,Eigen::Affine> O_T_EE_d;
  O_T_EE_d.linear()<<this->orientation_d_target_.toRotationMatrix();
  O_T_EE_d.translation()<<this->position_d_target_;  

  // ROS_INFO_STREAM(O_T_EE_d.matrix());
  std::array<double, 16> new_pose;
  Eigen::Map<Eigen::Matrix4d,Eigen::RowMajor>(new_pose.data())=O_T_EE_d.matrix();
  cartesian_pose_handle_->setCommand(new_pose);
  
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);  
}

void CartesianController::targetPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) 
{      
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

}  // namespace panda_controllers_extended

PLUGINLIB_EXPORT_CLASS(panda_controllers_extended::CartesianController,
                       controller_interface::ControllerBase)
