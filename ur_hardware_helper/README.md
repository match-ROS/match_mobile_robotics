# Ur Hardware Helper
## 1. Package overview
* `ur_hardware_helper`: Standalone launchfiles for different tasks of the ur
* `ur_controllers_extended`: Extension of the default universal robots controllers

# 2. Installation
### Install dependencies
```
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```
### Build packages
Use your standard build system to build the downloaded packages
