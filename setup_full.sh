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
sudo apt install ros-one-costmap-2d -y
sudo apt install ros-one-serial -y
sudo apt install ros-one-nav-core -y
sudo apt install ros-one-moveit-core -y
sudo apt install ros-one-ur-client-library -y
sudo apt install ros-one-moveit-ros-planning-interface -y
sudo apt install ros-one-mbf-msgs -y
sudo apt install ros-one-mir-actions -y
sudo apt install ros-one-navfn -y
sudo apt install ros-one-industrial-robot-status-interface -y
sudo apt install ros-one-move-base-msgs -y
sudo apt install ros-one-scaled-joint-trajectory-controller -y
sudo apt install ros-one-rospy-message-converter -y
sudo apt install ros-one-speed-scaling-interface -y
sudo apt install ros-one-speed-scaling-state-controller -y
sudo apt install ros-one-pass-through-controllers -y
sudo apt install libserial-dev -y
sudo apt install ros-one-rqt-* -y
sudo apt install ros-one-moveit-planners* -y
sudo apt install ros-one-moveit-ros* -y
apt install ros-one-ros-control* -y

