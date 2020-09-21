# 0. Package origin
This package came originally from (mir_robot)[https://github.com/dfki-ric/mir_robot].

# 1. Package overview
* `mir_actions`: Action definitions for the MiR robot
* `mir_description`: URDF description of the MiR robot
* `mir_dwb_critics`: Plugins for the dwb_local_planner used in Gazebo
* `mir_driver`: A reverse ROS bridge for the MiR robot
* `mir_gazebo`: Simulation specific launch and configuration files for the MiR robot
* `mir_msgs`: Message definitions for the MiR robot
* `mir_navigation`: move_base launch and configuration files
* `mir_helper`  : Tools for some walkarounds and better usage auf the mir plattform
* `mir_launcher`  : Standalone launchfiles for some different tasks

# 2. Installation
## Install dependencies
```
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```
## Build packages
Use your standard build system to build the downloaded packages