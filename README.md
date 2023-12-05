# ur5 slave demo environment
source ~/catkin_ur5/devel/setup.zsh

# ur5 ros demo
alias ur5_driver="roslaunch ur_robot_driver ur5_bringup.launch limited:=true robot_ip:=192.168.0.100"
alias ur5_moveit="roslaunch ur5_moveit_config moveit_planning_execution.launch limited:=true"
alias ur5_slave="rosrun slave_ur slave_ur"
alias omni_driver="roslaunch omni_common omni_state.launch"
alias ur5_rviz="roslaunch ur5_moveit_config moveit_rviz.launch config:=true"
