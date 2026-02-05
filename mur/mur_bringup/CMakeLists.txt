cmake_minimum_required(VERSION 3.0.2)
project(mur_bringup)

find_package(catkin REQUIRED)

catkin_package()

include_directories()

## Mark other files for installation (e.g. launch and config files)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

