#include <match_calibrated_ur_fk/match_ur_calibration_consumer.h>

namespace ur_launch_hardware
{
  MatchURCalibrationConsumer::MatchURCalibrationConsumer() : calibrated_(false)
  {
    this->dh_d_list_ = std::vector<double>();
    this->dh_a_list_ = std::vector<double>();
    this->dh_theta_list_ = std::vector<double>();
    this->dh_alpha_list_ = std::vector<double>();
  }

  bool MatchURCalibrationConsumer::consume(std::shared_ptr<urcl::primary_interface::PrimaryPackage> product)
  {
    auto kin_info = std::dynamic_pointer_cast<urcl::primary_interface::KinematicsInfo>(product);
    if (kin_info != nullptr)
    {
      ROS_INFO_STREAM(product->toString().c_str());
      // DHRobot my_robot;
      for (size_t i = 0; i < kin_info->dh_a_.size(); ++i)
      {
      //   my_robot.segments_.push_back(
      //       DHSegment(kin_info->dh_d_[i], kin_info->dh_a_[i], kin_info->dh_theta_[i], kin_info->dh_alpha_[i]));
        this->dh_d_list_.push_back(kin_info->dh_d_[i]);
        this->dh_a_list_.push_back(kin_info->dh_a_[i]);
        this->dh_theta_list_.push_back(kin_info->dh_theta_[i]);
        this->dh_alpha_list_.push_back(kin_info->dh_alpha_[i]);

        ROS_INFO_STREAM("d: " << kin_info->dh_d_[i]);
        ROS_INFO_STREAM("a: " << kin_info->dh_a_[i]);
        ROS_INFO_STREAM("theta: " << kin_info->dh_theta_[i]);
        ROS_INFO_STREAM("alpha: " << kin_info->dh_alpha_[i]);
      }
      // Calibration calibration(my_robot);
      // calibration.correctChain();

      // calibration_parameters_ = calibration.toYaml();
      // calibration_parameters_["kinematics"]["hash"] = kin_info->toHash();
      calibrated_ = true;
    }
    return true;
  }

  // urcl::primary_interface::KinematicsInfo MatchURCalibrationConsumer::getCalibrationParameters() const
  // {
  //   if (!calibrated_)
  //   {
  //     throw(std::runtime_error("Cannot get calibration, as no calibration data received yet"));
  //   }
  //   // return this->calibration_parameters_;
  // }
}  // namespace ur_calibration
