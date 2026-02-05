#include <ros/ros.h>

#include "cartesian_velocity_controller/cartesian_velocity_controller.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cartesian_velocity_controller", ros::init_options::NoSigintHandler);

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try
  {
    cartesian_velocity_controller::CartesianVelocityController controller(nh, pnh);
    controller.start();
    ros::spin();
  }
  catch (const std::exception& ex)
  {
    ROS_FATAL_STREAM("Failed to start CartesianVelocityController: " << ex.what());
    return 1;
  }

  return 0;
}

