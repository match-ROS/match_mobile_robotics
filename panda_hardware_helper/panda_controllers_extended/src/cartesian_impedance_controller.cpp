// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <panda_controllers_extended/cartesian_impedance_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <panda_controllers_extended/pseudo_inversion.h>

namespace panda_controllers_extended {

bool CartesianImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {      
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("enable_accelaration", this->enable_acc_)) {
    ROS_ERROR_STREAM("CartesianImpedanceController: Could not read parameter enable_accelaration");
    return false;
  }
  double rate;
  if (!node_handle.getParam("rate", rate)) {
    ROS_ERROR_STREAM("CartesianImpedanceController: Could not read parameter rate");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianImpedanceController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  sub_equilibrium_pose_ = node_handle.subscribe(
      "equilibrium_pose", 20, &CartesianImpedanceController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
      
  sub_imu_=node_handle.subscribe("imu",20,&CartesianImpedanceController::accelerationCallback,this,
      ros::TransportHints().reliable().tcpNoDelay());


  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle("dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<panda_controllers_extended::StiffnessConfig>>(
      node_handle);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&CartesianImpedanceController::complianceParamCallback, this, _1, _2));

  my_timer_=node_handle.createTimer(ros::Rate(rate),&CartesianImpedanceController::myUpdate,this);
  
  position_d_.setZero();
  acceleration_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  acceleration_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  tau_d_=Eigen::VectorXd(7);
  tau_d_.setZero();

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  return true;
}

void CartesianImpedanceController::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;
}


void CartesianImpedanceController::myUpdate(const ros::TimerEvent &)
{
   // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 49> mass_array= this->model_handle_->getMass();

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());  //Coriolis vector
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data()); //Jacobian matrix
  Eigen::Map<Eigen::Matrix<double, 7, 7>> mass(mass_array.data());  //Mass matrix

  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());  //Positions
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());  //Velocity
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data()); //Torque_desired 
  

  transform_=Eigen::Affine3d(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  
  Eigen::Vector3d position(transform_.translation());
  Eigen::Quaterniond orientation(transform_.linear());

  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  computeError(error,position,orientation);

 
  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7),tau_dyn(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  
  // Cartesian PD control with damping 
  tau_task << jacobian.transpose() *
              (-cartesian_stiffness_ * error 
               - cartesian_damping_ * (jacobian * dq));

  //Dynamik ff         
  // tau_dyn<< jacobian.transpose()*jacobian_transpose_pinv*acceleration_d_;
  // ROS_INFO_STREAM(std::endl<<"Torque:"<<std::endl<<tau_dyn);
  
  // nullspace PD control with damping 
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) +
                        nullspace_damping_* (dq_d_nullspace_-dq));
 
  // Desired torque
  // tau_d << tau_task + tau_nullspace + coriolis +tau_dyn ;
  tau_d << tau_task + tau_nullspace + coriolis;
  // Saturate torque rate to avoid discontinuities
  tau_d_ << saturateTorqueRate(tau_d, tau_J_d);
}
void CartesianImpedanceController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& /*period*/) {

  for (size_t i = 0; i < 7; ++i) 
  {
    joint_handles_[i].setCommand(tau_d_(i));
  }

 
  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  nullspace_damping_ =
      filter_params_ * nullspace_damping_target_ + (1.0 - filter_params_) * nullspace_damping_;
  
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  acceleration_d_ = filter_params_ * acceleration_d_target_ + (1.0 - filter_params_) * acceleration_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void CartesianImpedanceController::complianceParamCallback(
    panda_controllers_extended::StiffnessConfig& config,
    uint32_t /*level*/) 
  {        
  //Alloc cartesian stiffnes
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.Kx,0.0,0.0,0.0,config.Ky,0.0,0.0,0.0,config.Kz;
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.Kroll,0.0,0.0,0.0,config.Kpitch,0.0,0.0,0.0,config.Kyaw;
  
  //Alloc cartesian damping
  cartesian_damping_target_.setIdentity();
  cartesian_damping_target_.topLeftCorner(3, 3)
       << config.Dx,0.0,0.0,0.0,config.Dy,0.0,0.0,0.0,config.Dz;
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << config.Droll,0.0,0.0,0.0,config.Dpitch,0.0,0.0,0.0,config.Dyaw;
  
  //Alloc nullspace stiffness and damping
  nullspace_stiffness_target_ = config.nullspace_stiffness;
  nullspace_damping_target_= config.nullspace_damping;
}

void CartesianImpedanceController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) 
{
  //Set desired position
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  //Save orientation from last scope
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);

  //Set desired orientation
  orientation_d_target_.coeffs() << msg->pose.orientation.x, 
                                    msg->pose.orientation.y,
                                    msg->pose.orientation.z,
                                    msg->pose.orientation.w;
  //Flip orientation if nescessary                                  
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }

  // Predict the new joint angle state based on the transformation of cartesian error
  std::array<double, 42> jacobian_array =model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::MatrixXd jacobian_pinv;
  pseudoInverse(jacobian, jacobian_pinv);  
  Eigen::Matrix<double, 6, 1> error;
  computeError(error,position_d_target_,orientation_d_target_);
  this->dq_d_nullspace_=jacobian_pinv*error;
  this->q_d_nullspace_=this->q_d_nullspace_+this->dq_d_nullspace_;

}


void CartesianImpedanceController::accelerationCallback(const geometry_msgs::TwistStamped& msg)
{
  if(this->enable_acc_)
  {
    this->acceleration_d_target_<<msg.twist.linear.x,
                                  msg.twist.linear.y,
                                  msg.twist.linear.z,
                                  msg.twist.angular.x,
                                  msg.twist.angular.y,
                                  msg.twist.angular.z;
  }
}

void CartesianImpedanceController::computeError(Eigen::Matrix<double, 6, 1> &error, Eigen::Vector3d position,Eigen::Quaterniond orientation)
{
  error.head(3) << position - position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform_.linear() * error.tail(3);
}

}  // namespace panda_controllers_extended

PLUGINLIB_EXPORT_CLASS(panda_controllers_extended::CartesianImpedanceController,
                       controller_interface::ControllerBase)
