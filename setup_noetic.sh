sudo apt-get install ros-$ROS_DISTRO-ddynamic-reconfigure
sudo apt-get install python3-rosdep python3-wstool ros-noetic-ros
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
sudo apt install --no-install-recommends libasio-dev
git submodule update --init --recursive
cd ../..
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
