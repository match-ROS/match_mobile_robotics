# Panda Hardware Helper
## 1. Package overview
* `panda_hardware_helper`: Standalone launchfiles for some different tasks
* `panda_controllers_extended`: Extension of the default franka controllers

# 2. Installation
### Install dependencies
```
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```
### Build packages
Use your standard build system to build the downloaded packages
