#include <ros/ros.h>

#include "teleoperation/teleop_slave_controller.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleoperation_slave_cartesian_vv_controller");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try
  {
    teleoperation::TeleopSlaveController ctrl(nh, pnh);
    ctrl.start();
    ros::spin();
    ctrl.stop();
  }
  catch (const std::exception& ex)
  {
    ROS_FATAL("Failed to start teleoperation slave controller: %s", ex.what());
    return 1;
  }

  return 0;
}

