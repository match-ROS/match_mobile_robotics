cmake_minimum_required(VERSION 3.0.2)
project(teleoperation)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  moveit_core
  moveit_ros_planning
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs tf2_eigen moveit_core moveit_ros_planning
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# =============================================================================
# Component libraries
# =============================================================================

add_library(teleop_jacobian_solver
  src/components/jacobian_solver.cpp
)
target_link_libraries(teleop_jacobian_solver
  ${catkin_LIBRARIES}
)

add_library(teleop_robot_state_manager
  src/components/robot_state_manager.cpp
)
add_dependencies(teleop_robot_state_manager
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(teleop_robot_state_manager
  ${catkin_LIBRARIES}
)

add_library(teleop_pid_controller
  src/components/pid_controller.cpp
)
target_link_libraries(teleop_pid_controller
  ${catkin_LIBRARIES}
)

add_library(teleop_joint_safety_limiter
  src/components/joint_safety_limiter.cpp
)
target_link_libraries(teleop_joint_safety_limiter
  ${catkin_LIBRARIES}
)

add_library(teleop_slave_controller
  src/teleop_slave_controller.cpp
)
target_link_libraries(teleop_slave_controller
  teleop_robot_state_manager
  teleop_jacobian_solver
  teleop_pid_controller
  teleop_joint_safety_limiter
  ${catkin_LIBRARIES}
)

# =============================================================================
# Nodes
# =============================================================================

add_executable(teleoperation_slave_cartesian_vv_controller
  src/teleop_slave_controller_node.cpp
)
add_dependencies(teleoperation_slave_cartesian_vv_controller
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(teleoperation_slave_cartesian_vv_controller
  teleop_slave_controller
  ${catkin_LIBRARIES}
)

add_executable(teleop_master_measured_state_publisher
  src/teleop_master_measured_state_publisher_node.cpp
)
add_dependencies(teleop_master_measured_state_publisher
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(teleop_master_measured_state_publisher
  ${catkin_LIBRARIES}
)

add_executable(teleop_master_haptic_controller
  src/teleop_master_haptic_controller_node.cpp
)
add_dependencies(teleop_master_haptic_controller
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(teleop_master_haptic_controller
  ${catkin_LIBRARIES}
)

add_executable(teleop_slave_twist_outer_loop
  src/teleop_slave_twist_outer_loop_node.cpp
)
add_dependencies(teleop_slave_twist_outer_loop
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(teleop_slave_twist_outer_loop
  teleop_pid_controller
  ${catkin_LIBRARIES}
)

add_executable(teleop_bimanual_coupling
  src/teleop_bimanual_coupling_node.cpp
)
add_dependencies(teleop_bimanual_coupling
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(teleop_bimanual_coupling
  ${catkin_LIBRARIES}
)

# =============================================================================
# Install
# =============================================================================

catkin_install_python(PROGRAMS
  scripts/teleop_master_state_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  #teleop_jacobian_solver
  #teleop_robot_state_manager
  #teleop_pid_controller
  #teleop_joint_safety_limiter
  #teleop_slave_controller
  #teleoperation_slave_cartesian_vv_controller
  teleop_master_measured_state_publisher
  teleop_master_haptic_controller
  teleop_slave_twist_outer_loop
  teleop_bimanual_coupling
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

