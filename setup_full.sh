# copy meshes to the local gazebo model folder
mkdir ~/.gazebo/models/meshes -p
cp match_gazebo/models/meshes/* /home/$USER/.gazebo/models/meshes

git submodule update --init --recursive
cd submodules/match_path_planning/splined_voronoi/nlopt/
mkdir build
cd build
cmake ..
make
sudo make install
cd ../../../../../..
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
catkin build
source devel/setup.bash

# install dependencies manually (this should usually be done through rosdep) 
sudo apt install ros-noetic-costmap-2d
sudo apt install ros-noetic-serial
sudo apt install ros-noetic-nav-core
sudo apt install ros-noetic-moveit-core
sudo apt install ros-noetic-ur-client-library
sudo apt install ros-noetic-moveit-ros-planning-interface
sudo apt install ros-noetic-mbf-msgs
sudo apt install ros-noetic-mir-actions
sudo apt install ros-noetic-navfn
sudo apt install ros-noetic-industrial-robot-status-interface
sudo apt install ros-noetic-move-base-msgs
sudo apt install ros-noetic-scaled-joint-trajectory-controller
sudo apt install ros-noetic-rospy-message-converter
sudo apt install ros-noetic-speed-scaling-interface
sudo apt install ros-noetic-speed-scaling-state-controller
sudo apt install ros-noetic-pass-through-controllers