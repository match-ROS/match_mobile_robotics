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
cd ../../../../../../..
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
catkin build
source devel/setup.bash

# install dependencies manually (this should usually be done through rosdep) 
sudo apt install ros-noetic-costmap-2d -y
sudo apt install ros-noetic-serial -y
sudo apt install ros-noetic-nav-core -y
sudo apt install ros-noetic-moveit-core -y
sudo apt install ros-noetic-ur-client-library -y
sudo apt install ros-noetic-moveit-ros-planning-interface -y
sudo apt install ros-noetic-mbf-msgs -y
sudo apt install ros-noetic-mir-actions -y
sudo apt install ros-noetic-navfn -y
sudo apt install ros-noetic-industrial-robot-status-interface -y
sudo apt install ros-noetic-move-base-msgs -y
sudo apt install ros-noetic-scaled-joint-trajectory-controller -y
sudo apt install ros-noetic-rospy-message-converter -y
sudo apt install ros-noetic-speed-scaling-interface -y
sudo apt install ros-noetic-speed-scaling-state-controller -y
sudo apt install ros-noetic-pass-through-controllers -y
sudo apt install libserial-dev -y
sudo apt install ros-noetic-rqt-* -y
sudo apt install ros-noetic-moveit-planners* -y
sudo apt install ros-noetic-moveit-ros* -y
sudo apt install ros-noetic-ros-control* -y
sudo apt install ros-noetic-moveit-commander -y
sudo apt install ros-noetic-pcl-ros -y
sudo apt install ros-noetic-tf2-sensor-msgs -y
