/**
 * @file object_manager.hpp
 * @brief Manager for dynamic collision objects in the MoveIt planning scene
 *
 * This file defines the ObjectManager class that allows:
 * - Adding/removing collision objects from the scene
 * - Moving objects via waypoints or velocity commands
 * - Managing motion sequences
 */

#ifndef SCENE_BUILDER_OBJECTS_OBJECT_MANAGER_HPP
#define SCENE_BUILDER_OBJECTS_OBJECT_MANAGER_HPP

#include "scene_builder/objects/object_state.hpp"
#include "scene_builder/animation/waypoint_animator.hpp"
#include "scene_builder/animation/velocity_controller.hpp"
#include "scene_builder/animation/motion_sequence.hpp"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <ros/node_handle.h>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace scene_builder
{
namespace objects
{

/**
 * @brief Manager for dynamic collision objects in the MoveIt planning scene
 *
 * ObjectManager allows:
 * - Adding/removing collision objects (boxes, spheres, meshes)
 * - Animating objects via waypoint sequences
 * - Controlling objects with velocity commands
 *
 * Thread-safe: all operations are protected by mutex.
 *
 * Note: Distance computation has been moved to distance::DistanceCalculator.
 * Use that class directly for robot-object distance queries.
 */
class ObjectManager
{
public:
  /**
   * @brief Constructor
   * @param nh ROS NodeHandle
   * @param move_group_name Name of the MoveGroup to use (e.g., "arm", "manipulator")
   */
  ObjectManager(const ros::NodeHandle& nh, const std::string& move_group_name);

  /**
   * @brief Adds or updates a collision object in the scene
   * @param object MoveIt collision object
   * @param error_message Error message on failure
   * @return true if operation succeeded, false otherwise
   */
  bool addObject(const moveit_msgs::CollisionObject& object, std::string& error_message);

  /**
   * @brief Queues a waypoint for an object's movement queue
   * @param object_id ID of the object to move
   * @param command Waypoint command with target pose and duration
   * @param error_message Error message on failure
   * @return true if waypoint was queued, false otherwise
   * @note Clears any active velocity command
   */
  bool queueWaypoint(const std::string& object_id, const WaypointCommand& command, std::string& error_message);

  /**
   * @brief Sets a waypoint sequence to be played automatically
   * @param object_id Object ID
   * @param sequence Sequence of waypoint commands to execute in order
   * @param loop true to repeat the sequence cyclically
   */
  void setWaypointSequence(const std::string& object_id,
                           const std::vector<WaypointCommand>& sequence,
                           bool loop);

  /**
   * @brief Applies a velocity command to an object
   * @param object_id ID of the object to control
   * @param command Velocity command (twist + timeout)
   * @param error_message Error message on failure
   * @return true if velocity was applied, false otherwise
   * @note Clears all queued waypoints
   */
  bool applyVelocity(const std::string& object_id, const VelocityCommand& command, std::string& error_message);

  /**
   * @brief Removes an object from the scene
   * @param object_id ID of the object to remove
   * @param error_message Error message on failure
   * @return true if object was removed, false otherwise
   */
  bool removeObject(const std::string& object_id, std::string& error_message);

  /**
   * @brief Clears all commands (waypoints and velocity) for an object
   * @param object_id Object ID
   */
  void clearCommands(const std::string& object_id);

  /**
   * @brief Gets the sequence state of an object
   * @param object_id Object ID
   * @param is_active Output: true if sequence is active
   * @param is_loop Output: true if looping is enabled
   * @param waypoints Output: waypoints in the sequence
   * @param current_index Output: current waypoint index
   * @return true if object exists
   */
  bool getSequenceState(const std::string& object_id,
                        bool& is_active,
                        bool& is_loop,
                        std::vector<WaypointCommand>& waypoints,
                        std::size_t& current_index);

  /**
   * @brief Gets the current pose of an object (public version)
   * @param object_id Object ID
   * @param pose_out Output pose
   * @return true if object exists
   */
  bool getObjectPosePublic(const std::string& object_id, geometry_msgs::Pose& pose_out);

  /**
   * @brief Returns the list of all objects in the scene
   * @return Vector of CollisionObjects
   */
  std::vector<moveit_msgs::CollisionObject> listObjects();

  /**
   * @brief Updates positions of moving objects
   * @param now Current timestamp
   * @param update_dt Delta time from last update (seconds)
   * @note Must be called periodically to update animations and velocity commands
   */
  void update(const ros::Time& now, double update_dt);

  /**
   * @brief Loads predefined objects from a ROS parameter
   * @param nh NodeHandle to search for parameter
   * @param param Parameter name containing object definitions
   * @param loaded_objects Optional output: vector of loaded objects
   * @note Parameter must be a YAML dictionary with specific structure
   */
  void loadObjectsFromParameter(const ros::NodeHandle& nh,
                                const std::string& param,
                                std::vector<moveit_msgs::CollisionObject>* loaded_objects = nullptr);

  // Configuration setters
  void setMaxVelocityNorm(double value);
  void setDefaultVelocityTimeout(double value);

  /**
   * @brief Gets the planning frame name
   * @return Planning frame from MoveGroup
   */
  std::string getPlanningFrame() const { return move_group_.getPlanningFrame(); }

private:
  /**
   * @brief Internally moves an object to a new pose
   * @param object_id Object ID
   * @param pose New pose
   * @param error_message Error message
   * @return true if movement succeeded
   */
  bool moveObjectInternal(const std::string& object_id,
                          const geometry_msgs::Pose& pose,
                          std::string& error_message);

  /**
   * @brief Gets the current pose of an object from the scene
   * @param object_id Object ID
   * @param pose_out Output pose
   * @return true if object exists and pose was obtained
   */
  bool getObjectPose(const std::string& object_id, geometry_msgs::Pose& pose_out);

  /**
   * @brief Ensures that a state exists for the specified object
   * @param object_id Object ID
   * @param state_out Pointer to state (output)
   * @return true if state exists or was created
   */
  bool ensureObjectState(const std::string& object_id, ObjectState*& state_out);

  // Private members
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  ros::NodeHandle nh_;
  mutable std::mutex mutex_;
  std::unordered_map<std::string, ObjectState> object_states_;

  // Animation components
  animation::WaypointAnimator waypoint_animator_;
  animation::VelocityController velocity_controller_;
  animation::MotionSequenceManager sequence_manager_;
};

}  // namespace objects

// Backward compatibility: export types to scene_builder namespace
using objects::ObjectManager;
using objects::WaypointCommand;
using objects::VelocityCommand;
using objects::ObjectState;

}  // namespace scene_builder

#endif  // SCENE_BUILDER_OBJECTS_OBJECT_MANAGER_HPP
