#  Reference 
https://github.com/hku-mars/FAST_LIO
https://github.com/ZJU-FAST-Lab/ego-planner

https://github.com/qhlai/hl_ws/tree/master/jaka

# Omni Setup
## setup_files/99-3dsystems.rules
Need to create a Udev rule to specify the ownership and permissions for the USB device. 
/etc/udev/rules.d/99-3dsystems.rules

## setup_files/libPhantomIOLib42.so
This file is unique to the 2022 and 2019 lib given by 3dsystems. USE IT.
/usr/lib/libPhantomIOLib42.so

https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US


# Realsense
D345i相机安装完驱动提示Cannot access /sys/class/video4linux
https://blog.csdn.net/qq_45049500/article/details/130035335

# 4060 Driver on Laptop
in BIOS, find the display/graphics driver setting, select Discrete which represents the external GPU. It fixed the ubuntu can't find the external monitor problem.

# UR external Control
You can also use sshfs command (on Linux) to mount remote UR directory (I have e-series and it works).

sshfs root@192.168.1.50:/ /home/tm/remoteURDir
where:

    root is UR user (default password is: easybot)
    192.168.1.50 is UR IP address
    / is root directory on UR
    /home/tm/remoteURDir is directory on local computer


https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases

https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md

# R3LIVEE
R3LIVE: A Robust, Real-time, RGB-colored, LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package
https://ieeexplore.ieee.org/document/9811935
https://github.com/hku-mars/r3live

# RTabMAP
http://wiki.ros.org/rtabmap_ros/TutorialsNoetic/HandHeldMapping
http://wiki.ros.org/rtabmap
https://github.com/introlab/rtabmap_ros
https://blog.csdn.net/weixin_44580210/article/details/89789416
https://github.com/introlab/rtabmap/wiki/Installation#ubuntu


# Setup By GPT
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
