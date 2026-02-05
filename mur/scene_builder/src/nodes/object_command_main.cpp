/**
 * @file object_command_main.cpp
 * @brief Main entry point for the object command node
 */

#include "scene_builder/nodes/object_command_node.hpp"

#include <ros/ros.h>

#include <exception>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_command_node");

  ros::NodeHandle nh;

  try
  {
    scene_builder::nodes::ObjectCommandNode node(nh);

    ROS_INFO("Object Command Node started successfully");

    ros::waitForShutdown();
  }
  catch (const std::exception& ex)
  {
    ROS_FATAL_STREAM("Failed to start ObjectCommandNode: " << ex.what());
    return 1;
  }
  catch (...)
  {
    ROS_FATAL("Failed to start ObjectCommandNode due to unknown error");
    return 1;
  }

  return 0;
}

