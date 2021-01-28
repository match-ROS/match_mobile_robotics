cmake_minimum_required(VERSION 2.8.3)
project(mir_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp  
  tf2
  tf2_ros
  tf2_geometry_msgs 
  actionlib_msgs
  diagnostic_msgs
  dynamic_reconfigure
  geometry_msgs
  mir_msgs
  move_base_msgs
  nav_msgs
  rosgraph_msgs
  roslaunch
  rospy
  rospy_message_converter
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    diagnostic_msgs
    dynamic_reconfigure
    geometry_msgs
    mir_msgs
    move_base_msgs
    nav_msgs
    rosgraph_msgs
    rospy_message_converter
    sensor_msgs
    std_msgs
    tf
    visualization_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(map_odom_broadcaster nodes/helper/map_odom_broadcaster.cpp)
add_executable(map_base_broadcaster nodes/helper/map_base_broadcaster.cpp)
add_executable(pose_stamper nodes/helper/pose_stamper.cpp)
add_executable(twist_stamper nodes/helper/twist_stamper.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(map_odom_broadcaster
  ${catkin_LIBRARIES}
)
target_link_libraries(map_base_broadcaster
  ${catkin_LIBRARIES}
)
target_link_libraries(pose_stamper
  ${catkin_LIBRARIES}
)
target_link_libraries(twist_stamper
  ${catkin_LIBRARIES}
)







#############
## Install ##
#############

catkin_install_python(PROGRAMS
  nodes/fake_mir_joint_publisher.py
  nodes/mir_bridge.py
  nodes/rep117_filter.py
  nodes/tf_remove_child_frames.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

roslaunch_add_file_check(launch)
