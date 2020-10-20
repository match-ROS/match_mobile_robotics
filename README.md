# Helper Tools
## 1. Repository overview
* `general_hardware_helper`: Contains helper packages that can be used for generic robots
* `mir_hardware_helper`: Contains helper packages that can be used for Mir100/Mir200
* `ur_hardware_helper`: Contains helper packages that can be used for Ur5
* `panda_hardware_helper`: Contains helper packages that can be used for panda robot
* `miranda_hardware_helper`: Contains helper packages that can be used for the combination of panda and mir
* `mur_hardware_helper`: Contains helper packages that can be used for the combination of ur and mir

# 2. Installation
### Install dependencies
```
git clone https://github.com/ibMH/Helper_Tools.git
git submodule init
git submodule update
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```
### Build packages
Use your standard build system to build the downloaded packages

## 3. External doc:
[Ur dashboard](https://s3-eu-west-1.amazonaws.com/ur-support-site/15690/Dashboard_Server_CB-Series.pdf)  
[UR Ros driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/64ab483b550a1c079c70162d2d3c2eb21ecde76e)  
[UR Robot](https://github.com/fmauch/universal_robot/tree/3ebf8070ad0869c264fc3df9185fe1865773b2b4)  
[Franka (Panda)](https://frankaemika.github.io/docs/index.html)  
