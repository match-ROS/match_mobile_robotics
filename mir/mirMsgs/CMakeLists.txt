cmake_minimum_required(VERSION 3.5.1)
project(mirMsgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
  FILES
    SafetyStatus.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
)
