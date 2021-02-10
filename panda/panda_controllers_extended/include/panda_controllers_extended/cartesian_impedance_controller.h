// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <eigen3/Eigen/Dense>

#include <panda_controllers_extended/StiffnessConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace panda_controllers_extended {

/**
 * @brief A class that provides cartesian impedance control based on the 
 * [franka_example_controllers](https://github.com/frankaemika/franka_ros/tree/kinetic-devel/franka_example_controllers)
 * 
 */
class CartesianImpedanceController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
    /**
     * @brief Initializes the controller
     * 
     * @param robot_hw The Hardware Interface the controller is working with
     * @param node_handle The Nodehandle for handling ros ressources
     * @return true Initialisation done
     * @return false Initialisation error
     */
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  
  /**
   * @brief  Starting the controller. E.g setting the target pose to the current one
   * 
   */
  void starting(const ros::Time&) override;
  /**
   * @brief Updates the control scope. Determines the desired torques by cartesian impedance control
   * 
   * @param period Control period
   */
  void update(const ros::Time&, const ros::Duration& period) override;


 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  ros::Timer my_timer_;

  double filter_params_{0.005};
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};
  double nullspace_damping_{9.0};
  double nullspace_damping_target_{9.0};
  const double delta_tau_max_{1.0};

  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Matrix<double, 7, 1> dq_d_nullspace_;
  Eigen::VectorXd tau_d_;

  bool enable_acc_;

  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  Eigen::Matrix<double,6,1> acceleration_d_;
  
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;
  Eigen::Matrix<double,6,1> acceleration_d_target_;
  
  Eigen::Affine3d transform_;

  std::unique_ptr<dynamic_reconfigure::Server<panda_controllers_extended::StiffnessConfig>>dynamic_server_compliance_param_;
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
 
  // Equilibrium pose subscriber
  ros::Subscriber sub_equilibrium_pose_;
  ros::Subscriber sub_imu_;

  void myUpdate(const ros::TimerEvent &);

  void computeError(Eigen::Matrix<double, 6, 1> &error, Eigen::Vector3d position,Eigen::Quaterniond orientation);
  // Dynamic reconfigure
 
  void complianceParamCallback(panda_controllers_extended::StiffnessConfig& config,
                               uint32_t level);


  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  void accelerationCallback(const geometry_msgs::TwistStamped& msg);
};

}  // namespace panda_controllers_extended
