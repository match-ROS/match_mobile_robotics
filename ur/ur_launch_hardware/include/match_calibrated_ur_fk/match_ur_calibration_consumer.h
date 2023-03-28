#ifndef UR_LAUNCH_HARDWARE_MATCH_UR_CALIBRATION_CONSUMER_H_INCLUDED
#define UR_LAUNCH_HARDWARE_MATCH_UR_CALIBRATION_CONSUMER_H_INCLUDED
#include "ros/ros.h"

#include <ur_client_library/comm/pipeline.h>

#include <ur_client_library/primary/robot_state/kinematics_info.h>

#include <vector>

namespace ur_launch_hardware
{
class MatchURCalibrationConsumer : public urcl::comm::IConsumer<urcl::primary_interface::PrimaryPackage>
{
public:
  MatchURCalibrationConsumer();
  virtual ~MatchURCalibrationConsumer() = default;

  virtual bool consume(std::shared_ptr<urcl::primary_interface::PrimaryPackage> product);

  bool isCalibrated() const
  {
    return calibrated_;
  }

  // urcl::primary_interface::KinematicsInfo getCalibrationParameters() const;

private:
  bool calibrated_;

  std::vector<double> dh_d_list_;
  std::vector<double> dh_a_list_;
  std::vector<double> dh_theta_list_;
  std::vector<double> dh_alpha_list_;

  // urcl::primary_interface::KinematicsInfo calibration_parameters_;
};
}  // namespace ur_launch_hardware
#endif  // ifndef UR_LAUNCH_HARDWARE_MATCH_UR_CALIBRATION_CONSUMER_H_INCLUDED
