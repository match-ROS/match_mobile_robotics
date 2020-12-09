// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <franka_hw/franka_cartesian_command_interface.h>

namespace panda_controllers_extended {

class CartesianController
    : public controller_interface::MultiInterfaceController<franka_hw::FrankaPoseCartesianInterface,
                                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  ros::Subscriber target_pose_sub_;

  double filter_params_{0.005};

  Eigen::Vector3d position_d_target_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_target_;
  Eigen::Quaterniond orientation_d_;

  franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;

  std::array<double, 16> initial_pose_{};
  std::array<double, 16> desired_pose_{};

  void targetPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
};

}  // namespace panda_controllers_extended
