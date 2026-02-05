/**
 * @file object_command_node.hpp
 * @brief ROS node for managing collision object commands
 *
 * This node provides ROS interfaces for controlling dynamic collision objects
 * via topics and services.
 */

#ifndef SCENE_BUILDER_NODES_OBJECT_COMMAND_NODE_HPP
#define SCENE_BUILDER_NODES_OBJECT_COMMAND_NODE_HPP

#include "scene_builder/objects/object_manager.hpp"

#include <scene_builder/ObjectCommand.h>
#include <scene_builder/ObjectVelocityCommand.h>
#include <scene_builder/SetMotionSequence.h>
#include <scene_builder/GetMotionSequence.h>
#include <scene_builder/ClearMotionSequence.h>
#include <scene_builder/ListObjects.h>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <moveit_msgs/CollisionObject.h>

#include <memory>
#include <string>
#include <vector>

namespace scene_builder
{
namespace nodes
{

/**
 * @brief ROS node for managing collision object commands
 *
 * ObjectCommandNode provides a complete ROS interface for controlling dynamic
 * collision objects in the MoveIt planning scene.
 *
 * SUBSCRIBED TOPICS:
 * - add_object (moveit_msgs/CollisionObject): Adds/updates objects
 * - object_command (ObjectCommand): Waypoint commands for objects
 * - object_velocity_command (ObjectVelocityCommand): Velocity commands
 * - object_animation (geometry_msgs/PoseArray): Animation sequences
 *
 * PUBLISHED TOPICS:
 * - objects (moveit_msgs/CollisionObject): List of objects present
 *
 * SERVICES:
 * - ~set_motion_sequence: Sets a waypoint sequence
 * - ~get_motion_sequence: Gets sequence state
 * - ~clear_motion_sequence: Clears a sequence
 * - ~list_objects: Lists all objects
 *
 * ROS PARAMETERS:
 * - ~move_group (string): MoveGroup name (default: "arm")
 * - ~default_objects_param (string): Parameter with default objects
 * - ~update_rate (double): Update frequency in Hz (default: 60.0)
 * - ~default_command_duration (double): Default command duration (default: 0.5s)
 * - ~motion_sequences_param (string): Parameter with motion sequences
 * - ~autostart_loops (bool): Auto-start loops at startup (default: false)
 */
class ObjectCommandNode
{
public:
  /**
   * @brief Constructor - Initializes the node with all components
   * @param nh ROS NodeHandle
   */
  explicit ObjectCommandNode(const ros::NodeHandle& nh);

private:
  /**
   * @brief Loads ROS parameters
   */
  void loadParameters();

  /**
   * @brief Loads predefined motion sequences from configuration
   */
  void loadMotionSequences();

  /**
   * @brief Sets up publishers and subscribers
   */
  void setupPublishersAndSubscribers();

  /**
   * @brief Sets up timers for periodic updates
   */
  void setupTimers();

  /**
   * @brief Callback for adding/updating collision objects
   */
  void collisionObjectCallback(const moveit_msgs::CollisionObject::ConstPtr& msg);

  /**
   * @brief Callback for waypoint commands
   */
  void objectCommandCallback(const scene_builder::ObjectCommand::ConstPtr& msg);

  /**
   * @brief Callback for velocity commands
   */
  void velocityCommandCallback(const scene_builder::ObjectVelocityCommand::ConstPtr& msg);

  /**
   * @brief Callback for animation sequences
   */
  void animationSequenceCallback(const geometry_msgs::PoseArray::ConstPtr& msg);

  /**
   * @brief Timer callback - Updates all object positions
   */
  void updateTimerCallback(const ros::TimerEvent& event);

  /**
   * @brief Service handler for SetMotionSequence
   */
  bool handleSetMotionSequence(SetMotionSequence::Request& req, SetMotionSequence::Response& res);

  /**
   * @brief Service handler for GetMotionSequence
   */
  bool handleGetMotionSequence(GetMotionSequence::Request& req, GetMotionSequence::Response& res);

  /**
   * @brief Service handler for ClearMotionSequence
   */
  bool handleClearMotionSequence(ClearMotionSequence::Request& req, ClearMotionSequence::Response& res);

  /**
   * @brief Service handler for ListObjects
   */
  bool handleListObjects(ListObjects::Request& req, ListObjects::Response& res);

  // ROS interfaces
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::CallbackQueue callback_queue_;
  ros::AsyncSpinner spinner_;

  // Core manager
  std::unique_ptr<objects::ObjectManager> manager_;

  // Subscribers
  ros::Subscriber add_object_sub_;
  ros::Subscriber waypoint_sub_;
  ros::Subscriber velocity_sub_;
  ros::Subscriber animation_sub_;

  // Publishers
  ros::Publisher object_list_pub_;

  // Service servers
  ros::ServiceServer set_motion_sequence_srv_;
  ros::ServiceServer get_motion_sequence_srv_;
  ros::ServiceServer clear_motion_sequence_srv_;
  ros::ServiceServer list_objects_srv_;

  // Timers
  ros::Timer update_timer_;

  // Parameters
  std::string move_group_ = "arm";
  std::string default_objects_param_ = "default_objects";
  double update_rate_ = 60.0;
  double default_command_duration_ = 0.5;
  std::string motion_sequences_param_;
  bool autostart_loops_ = false;
};

}  // namespace nodes
}  // namespace scene_builder

#endif  // SCENE_BUILDER_NODES_OBJECT_COMMAND_NODE_HPP

