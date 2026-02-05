/**
 * @file distance_monitor_main.cpp
 * @brief Main entry point for the distance monitor node
 */

#include "scene_builder/nodes/distance_monitor_node.hpp"

#include <ros/ros.h>

#include <exception>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "distance_monitor_node");

  // AsyncSpinner with 2 threads is required for PlanningSceneMonitor
  // to properly handle callbacks and scene updates
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try
  {
    scene_builder::nodes::DistanceMonitorNode node(nh, pnh);
    node.start();

    ROS_INFO("Distance Monitor Node started successfully");

    ros::waitForShutdown();
  }
  catch (const std::exception& ex)
  {
    ROS_FATAL_STREAM("Failed to start DistanceMonitorNode: " << ex.what());
    return 1;
  }
  catch (...)
  {
    ROS_FATAL("Failed to start DistanceMonitorNode due to unknown error");
    return 1;
  }

  return 0;
}

