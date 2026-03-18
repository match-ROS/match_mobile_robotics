cmake_minimum_required(VERSION 3.0.2)
project(teleoperation_simplified)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(teleop_master_admittance
  src/teleop_master_admittance_node.cpp
)
add_dependencies(teleop_master_admittance
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(teleop_master_admittance
  ${catkin_LIBRARIES}
)

add_executable(teleop_slave_pose_admittance
  src/teleop_slave_pose_admittance_node.cpp
)
add_dependencies(teleop_slave_pose_admittance
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(teleop_slave_pose_admittance
  ${catkin_LIBRARIES}
)

install(TARGETS
  teleop_master_admittance
  teleop_slave_pose_admittance
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
