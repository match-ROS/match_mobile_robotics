cmake_minimum_required(VERSION 3.0.2)
project(cartesian_velocity_controller)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  controller_manager_msgs
  moveit_core
  moveit_ros_planning
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  message_generation
  dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)

# Find fuzzylite library (optional - enables FuzzyGainScheduler)
find_library(FUZZYLITE_LIB NAMES fuzzylite PATHS /usr/local/lib)
find_path(FUZZYLITE_INCLUDE_DIR NAMES fl/Headers.h PATHS /usr/local/include)

if(FUZZYLITE_LIB AND FUZZYLITE_INCLUDE_DIR)
  set(FUZZYLITE_FOUND TRUE)
  message(STATUS "Found fuzzylite: ${FUZZYLITE_LIB}")
  message(STATUS "Fuzzylite include: ${FUZZYLITE_INCLUDE_DIR}")
  add_definitions(-DFUZZYLITE_AVAILABLE)
else()
  set(FUZZYLITE_FOUND FALSE)
  message(WARNING "fuzzylite not found - FuzzyGainScheduler will be disabled")
endif()

# Generate messages
# Note: POIDebugInfo.msg must be before PipelineDebug.msg since it depends on it
add_message_files(
  FILES
  POIDebugInfo.msg
  CartesianTrajectorySetpoint.msg
  EndEffectorState.msg
  PipelineDebug.msg
  JointVelocityFeedback.msg
  WholeBodyPrintDebug.msg
)

add_service_files(
  FILES
  QueryMap3D.srv
  GetFrameInfo.srv
  GetJacobian.srv
  ValidatePoses.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# Dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/ControllerTuning.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs std_srvs geometry_msgs sensor_msgs visualization_msgs controller_manager_msgs moveit_core moveit_ros_planning tf2 tf2_ros tf2_geometry_msgs tf2_eigen message_runtime dynamic_reconfigure
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Add fuzzylite include directory only if found
if(FUZZYLITE_FOUND)
  include_directories(${FUZZYLITE_INCLUDE_DIR})
endif()

# =============================================================================
# Component Libraries (built incrementally during refactoring)
# =============================================================================

# JacobianSolver component
add_library(jacobian_solver
  src/components/jacobian_solver.cpp
)
target_link_libraries(jacobian_solver
  ${catkin_LIBRARIES}
)

# GlobalPlanner component (new pipeline Level A)
# NOTE: do NOT name this library "global_planner" because that collides with the
# ROS navigation stack library `libglobal_planner.so` (pluginlib), which can
# break runtime loading of the nav_core planner `global_planner/GlobalPlanner`.
add_library(cvc_global_planner
  src/components/global_planner.cpp
)
target_link_libraries(cvc_global_planner
  ${catkin_LIBRARIES}
)

# LocalPlanner component (new pipeline Level B)
add_library(local_planner
  src/components/local_planner.cpp
)
target_link_libraries(local_planner
  robot_state_manager
  jacobian_solver
  ${catkin_LIBRARIES}
)

# JointSafetyLimiter component (new pipeline final stage)
add_library(joint_safety_limiter
  src/components/joint_safety_limiter.cpp
)
target_link_libraries(joint_safety_limiter
  ${catkin_LIBRARIES}
)

# PIDController component (Level D: PID control)
add_library(pid_controller
  src/components/pid_controller.cpp
)
target_link_libraries(pid_controller
  ${catkin_LIBRARIES}
)

# CartesianVelocityFilter component (Level C: Motion Generator)
add_library(velocity_filter
  src/velocity_filter.cpp
)
target_link_libraries(velocity_filter
  ${catkin_LIBRARIES}
)

# JointVelocityFilter component (joint-space smoothness stage before safety limiter)
add_library(joint_velocity_filter
  src/joint_velocity_filter.cpp
)
target_link_libraries(joint_velocity_filter
  ${catkin_LIBRARIES}
)

# MarkerPublisher component
add_library(marker_publisher
  src/components/marker_publisher.cpp
)
add_dependencies(marker_publisher
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(marker_publisher
  ${catkin_LIBRARIES}
)

# FeedbackPublisher component
add_library(feedback_publisher
  src/components/feedback_publisher.cpp
)
add_dependencies(feedback_publisher
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(feedback_publisher
  ${catkin_LIBRARIES}
)

# RobotStateManager component
add_library(robot_state_manager
  src/components/robot_state_manager.cpp
)
add_dependencies(robot_state_manager
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(robot_state_manager
  ${catkin_LIBRARIES}
)

# Map3D module (local EDT map for fast queries)
add_library(map3d
  src/map3d/voxel_grid_3d.cpp
  src/map3d/edt_calculator.cpp
  src/map3d/voxelizer_sphere.cpp
  src/map3d/planning_scene_sphere_reader.cpp
  src/map3d/map3d_manager.cpp
)
add_dependencies(map3d
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(map3d
  ${catkin_LIBRARIES}
)

# JointPositionGuard component (runtime joint limit guardrail)
add_library(joint_position_guard
  src/components/joint_position_guard.cpp
)
target_link_libraries(joint_position_guard
  ${catkin_LIBRARIES}
)

# RepulsionDataManager component
add_library(repulsion_data_manager
  src/components/repulsion_data_manager.cpp
)
add_dependencies(repulsion_data_manager
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(repulsion_data_manager
  robot_state_manager
  map3d
  ${catkin_LIBRARIES}
)

# JointWeightManager component
add_library(joint_weight_manager
  src/components/joint_weight_manager.cpp
)
target_link_libraries(joint_weight_manager
  ${catkin_LIBRARIES}
)

# =============================================================================
# Main Controller Node
# =============================================================================

add_executable(cartesian_velocity_node
  src/cartesian_velocity_node.cpp
  src/cartesian_velocity_controller.cpp
)

add_dependencies(cartesian_velocity_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(cartesian_velocity_node
  velocity_filter
  joint_velocity_filter
  jacobian_solver
  robot_state_manager
  joint_position_guard
  repulsion_data_manager
  marker_publisher
  feedback_publisher
  cvc_global_planner
  local_planner
  pid_controller
  joint_safety_limiter
  joint_weight_manager
  ${catkin_LIBRARIES}
)

add_executable(whole_body_print_controller_node
  src/whole_body_print_controller_node.cpp
)

add_dependencies(whole_body_print_controller_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(whole_body_print_controller_node
  ${catkin_LIBRARIES}
)

# =============================================================================
# Python Scripts Installation
# =============================================================================
catkin_install_python(PROGRAMS
  scripts/robot_interactive_control.py
  scripts/ensure_mur620_dual_arm_velocity_controllers.py
  scripts/ensure_mur620_single_arm_velocity_controller.py
  scripts/print_jacobian.py
  scripts/tcp_path_trajectory_manager.py
  scripts/whole_body_preflight_check.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/record_whole_body_print_bag.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# =============================================================================
# Install launch and config files
# =============================================================================
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
