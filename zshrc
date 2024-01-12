# If you come from bash you might have to change your $PATH.
# export PATH=$HOME/bin:/usr/local/bin:$PATH

# Path to your oh-my-zsh installation.
export ZSH="$HOME/.oh-my-zsh"

# Set name of the theme to load --- if set to "random", it will
# load a random theme each time oh-my-zsh is loaded, in which case,
# to know which specific one was loaded, run: echo $RANDOM_THEME
# See https://github.com/ohmyzsh/ohmyzsh/wiki/Themes
ZSH_THEME="robbyrussell"

plugins=(git)

source $ZSH/oh-my-zsh.sh

alias refresh="source ~/.zshrc"
alias python="python3"
alias terminator_start="terminator -l eight &"
alias teleop_works="cd /home/duandaxia/catkin_ur5/src/teleop/src"
# ur5 slave demo environment
source ~/catkin_ur5/devel/setup.zsh
teleop_works

# ur5 ros demo
alias ur5_driver="roslaunch ur_robot_driver ur5_bringup.launch limited:=true robot_ip:=192.168.0.100"
alias ur5_moveit="roslaunch ur5_moveit_config moveit_planning_execution.launch limited:=true"
alias ur5_slave="rosrun teleop apply_delta_button.py"
alias omni_driver="roslaunch omni_common omni_state.launch"
alias ur5_rviz="roslaunch ur5_moveit_config moveit_rviz.launch config:=true"

# jaka and force sensor environment
# source ~/catkin_master_slave_up/devel/setup.zsh

# jaka ros demo
alias jaka_driver="roslaunch jaka_ros_driver start.launch"
alias jaka_server="rosrun control_msgs jaka5_server"
alias jaka_rviz="roslaunch jaka5_config demo.launch"


# omni ros demo
alias omni_rviz="roslaunch omni_common omni.launch"
alias force_driver="roslaunch sunrise start_sensor.launch"


# rtabmap
# Alias for roslaunch realsense2_camera
alias start_realsense='roslaunch realsense2_camera rs_camera.launch align_depth:=true unite_imu_method:="linear_interpolation" enable_gyro:=true enable_accel:=true'
# Alias for rosrun imu_filter_madgwick
alias start_imu_filter='rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:="enu" /imu/data_raw:=/camera/imu /imu/data:=/rtabmap/imu'
# Alias for roslaunch rtabmap_launch
alias start_rtabmap='roslaunch rtabmap_launch rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu'

