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
sudo apt install ros-one-costmap-2d
sudo apt install ros-one-serial
sudo apt install ros-one-nav-core
sudo apt install ros-one-moveit-core
sudo apt install ros-one-ur-client-library
sudo apt install ros-one-moveit-ros-planning-interface
sudo apt install ros-one-mbf-msgs
sudo apt install ros-one-mir-actions
sudo apt install ros-one-navfn
sudo apt install ros-one-industrial-robot-status-interface
sudo apt install ros-one-move-base-msgs
sudo apt install ros-one-scaled-joint-trajectory-controller
sudo apt install ros-one-rospy-message-converter
sudo apt install ros-one-speed-scaling-interface
sudo apt install ros-one-speed-scaling-state-controller
sudo apt install ros-one-pass-through-controllers
sudo apt install libserial-dev