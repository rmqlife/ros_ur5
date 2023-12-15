#!/bin/bash

# Update package manager
sudo apt update

# Install required packages for RealSense
sudo apt-get install -y liberealsense2-dkms
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
sudo apt-get install -y librealsense2-utils librealsense2-dev librealsense2-dbg

# Remove any existing RealSense packages
dpkg -l | grep "realsense" | cut -d " " -f 3 | xargs sudo dpkg --purge

# Install ROS packages
sudo apt install -y ros-noetic-rtabmap*
sudo apt-get install -y libsqlite3-dev libpcl-dev libopencv-dev git cmake libproj-dev libqt5svg5-dev ros-noetic-rtabmap-ros ros-noetic-imu-filter-madgwick

# Build catkin workspace
catkin_make

# Source ROS setup files
source ./devel/setup.zsh

# Launch RealSense nodes
roslaunch realsense2_camera rs_camera.launch align_depth:=true
roslaunch rtabmap_launch rtabmap.launch \
    rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    rgb_topic:=/camera/color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    approx_sync:=false \
    wait_imu_to_init:=true \
    imu_topic:=/rtabmap/imu
