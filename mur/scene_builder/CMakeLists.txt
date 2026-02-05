cmake_minimum_required(VERSION 3.0.2)
project(scene_builder)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  tf2_geometry_msgs
  visualization_msgs
)

# Python setup for scene_control module (must be before generate_messages)
catkin_python_setup()

# Messages
add_message_files(FILES
  DistanceContact.msg
  DistanceInfo.msg
  ObjectCommand.msg
  ObjectVelocityCommand.msg
  RobotPointDistanceInfo.msg
  RobotPointsInfo.msg
)

# Services
add_service_files(FILES
  SetMotionSequence.srv
  GetMotionSequence.srv
  ClearMotionSequence.srv
  ListObjects.srv
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_core ${PROJECT_NAME}_objects ${PROJECT_NAME}_animation ${PROJECT_NAME}_distance
  CATKIN_DEPENDS geometry_msgs message_runtime moveit_core moveit_msgs moveit_ros_planning_interface roscpp rospy std_msgs tf2_geometry_msgs visualization_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

###########
## Build ##
###########

# Core library - reusable pose/yaml utilities
add_library(${PROJECT_NAME}_core
  src/core/pose_utils.cpp
  src/core/yaml_parser.cpp
  src/core/planning_scene_holder.cpp
)
add_dependencies(${PROJECT_NAME}_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_core ${catkin_LIBRARIES})

# Animation library - waypoint/velocity/sequence controllers
add_library(${PROJECT_NAME}_animation
  src/animation/waypoint_animator.cpp
  src/animation/velocity_controller.cpp
  src/animation/motion_sequence.cpp
)
add_dependencies(${PROJECT_NAME}_animation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_animation ${PROJECT_NAME}_core ${catkin_LIBRARIES})

# Objects library - object management components
add_library(${PROJECT_NAME}_objects
  src/objects/collision_object_factory.cpp
  src/objects/object_manager.cpp
)
add_dependencies(${PROJECT_NAME}_objects ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_objects
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_animation
  ${catkin_LIBRARIES}
)

# Distance library - distance calculation and visualization
add_library(${PROJECT_NAME}_distance
  src/distance/distance_calculator.cpp
  src/distance/distance_visualizer.cpp
  src/distance/velocity_estimator.cpp
  src/distance/robot_point_tracker.cpp
)
add_dependencies(${PROJECT_NAME}_distance ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_distance ${PROJECT_NAME}_core ${catkin_LIBRARIES})

# Executables
add_executable(distance_monitor_node
  src/nodes/distance_monitor_node.cpp
  src/nodes/distance_monitor_main.cpp
)
add_dependencies(distance_monitor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(distance_monitor_node
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_objects
  ${PROJECT_NAME}_distance
  ${catkin_LIBRARIES}
)

add_executable(object_command_node
  src/nodes/object_command_node.cpp
  src/nodes/object_command_main.cpp
)
add_dependencies(object_command_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(object_command_node
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_objects
  ${PROJECT_NAME}_animation
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Python scripts
catkin_install_python(PROGRAMS
  scripts/move_to_pose.py
  scripts/scene_interactive_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Libraries and executables
install(TARGETS
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_animation
  ${PROJECT_NAME}_objects
  ${PROJECT_NAME}_distance
  distance_monitor_node
  object_command_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Config, launch, and msg directories
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY msg/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg)
