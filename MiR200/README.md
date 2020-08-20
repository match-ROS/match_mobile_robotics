# 1. Package overview
* `mir_actions`: Action definitions for the MiR robot
* `mir_description`: URDF description of the MiR robot
* `mir_dwb_critics`: Plugins for the dwb_local_planner used in Gazebo
* `mir_driver`: A reverse ROS bridge for the MiR robot
* `mir_gazebo`: Simulation specific launch and configuration files for the MiR robot
* `mir_msgs`: Message definitions for the MiR robot
* `mir_navigation`: move_base launch and configuration files


# 2. Installation
The instructions below use the ROS distro `kinetic` as an example.

### Preliminaries
If you haven't already installed ROS on your PC, you need to add the ROS apt
repository. This step is necessary for either binary or source install.

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq
```

### Binary install
For a binary install, it suffices to run this command:
```bash
sudo apt install ros-kinetic-mir-robot
sudo apt install ros-kinetic-navigation
sudo apt install ros-kinetic-plotjuggler
```
See the tables at the end of this README for a list of ROS distros for which
binary packages are available.

### Source install
For a source install, run the commands below instead of the command from the
"binary install" section.
```bash
cd ~/catkin_ws/src/
```

### Clone mir_robot into the catkin workspace
```bash
git clone https://github.com/Jryl/MIR_SIM_Test.git
```
### Install all dependencies
```bash
sudo apt-get update -qq
sudo apt-get install -qq -y python-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths ./ -i -y --rosdistro kinetic
```

### Build all packages in the catkin workspace
```bash
source /opt/ros/kinetic/setup.bash
catkin_init_workspace
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebugInfo
```

# 3. Gazebo demo (existing map)
```bash
roslaunch mir_navigation mir_start.launch
```

Now, you can use the "2D Nav Goal" tool in RViz to set a navigation goal for move_base.

# 4. Gazebo demo (mapping)
```bash
roslaunch mir_navigation mir_mapping.launch
```

# 5. Teleoperate the robot with keyboard (optional)
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

# 6. Compile the python file
Find the file (.../mir_driver/nodes) and open the terminal in this folder
```bash
chmod +x rep117_filter.py
```
