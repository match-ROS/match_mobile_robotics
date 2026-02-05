/**
 * @file planning_scene_holder.cpp
 * @brief Implementation of the PlanningSceneHolder singleton
 */

#include "scene_builder/core/planning_scene_holder.hpp"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ros/console.h>

#include <stdexcept>

namespace scene_builder
{
namespace core
{

PlanningSceneHolder& PlanningSceneHolder::instance()
{
  static PlanningSceneHolder instance;
  return instance;
}

bool PlanningSceneHolder::initialize(const ros::NodeHandle& nh,
                                     const std::string& robot_description)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Already initialized - nothing to do
  if (initialized_ && monitor_)
  {
    return true;
  }

  // Resolve the robot_description parameter name
  std::string resolved_description = nh.resolveName(robot_description);
  if (!ros::param::has(resolved_description))
  {
    // Try to find the parameter in other namespaces
    std::string search_param;
    if (nh.searchParam(robot_description, search_param))
    {
      resolved_description = search_param;
    }
    else if (ros::param::search(robot_description, search_param))
    {
      resolved_description = search_param;
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("planning_scene_holder",
                            "Parameter '" << robot_description << "' not found in parameter server");
      return false;
    }
  }

  // Configure the RobotModelLoader
  robot_model_loader::RobotModelLoader::Options options(resolved_description);
  options.robot_description_ = resolved_description;

  // Load the robot model
  auto loader = std::make_shared<robot_model_loader::RobotModelLoader>(options);
  if (!loader->getModel())
  {
    ROS_ERROR_STREAM_NAMED("planning_scene_holder",
                          "Failed to load robot model from parameter '" << resolved_description << "'");
    return false;
  }

  // Create the PlanningSceneMonitor
  monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(loader);
  if (!monitor_->getPlanningScene())
  {
    ROS_ERROR_NAMED("planning_scene_holder", "Failed to create PlanningSceneMonitor");
    monitor_.reset();
    return false;
  }

  // Start the monitors to keep the scene updated
  monitor_->startSceneMonitor();           // Monitor scene changes
  monitor_->startWorldGeometryMonitor();   // Monitor world objects
  monitor_->startStateMonitor();           // Monitor robot state
  monitor_->providePlanningSceneService(); // Provide services for queries

  initialized_ = true;
  ROS_INFO_NAMED("planning_scene_holder", "PlanningSceneMonitor initialized successfully");

  return true;
}

planning_scene_monitor::PlanningSceneMonitorPtr PlanningSceneHolder::getMonitor()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return monitor_;
}

planning_scene_monitor::LockedPlanningSceneRO PlanningSceneHolder::getSceneRO()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_ || !monitor_)
  {
    throw std::runtime_error("PlanningSceneHolder not initialized. Call initialize() first.");
  }
  return planning_scene_monitor::LockedPlanningSceneRO(monitor_);
}

planning_scene_monitor::LockedPlanningSceneRW PlanningSceneHolder::getSceneRW()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_ || !monitor_)
  {
    throw std::runtime_error("PlanningSceneHolder not initialized. Call initialize() first.");
  }
  return planning_scene_monitor::LockedPlanningSceneRW(monitor_);
}

bool PlanningSceneHolder::isInitialized() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return initialized_ && monitor_ != nullptr;
}

std::string PlanningSceneHolder::getPlanningFrame() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_ || !monitor_ || !monitor_->getPlanningScene())
  {
    return "";
  }
  return monitor_->getPlanningScene()->getPlanningFrame();
}

moveit::core::RobotModelConstPtr PlanningSceneHolder::getRobotModel() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_ || !monitor_ || !monitor_->getPlanningScene())
  {
    return nullptr;
  }
  return monitor_->getPlanningScene()->getRobotModel();
}

}  // namespace core
}  // namespace scene_builder

