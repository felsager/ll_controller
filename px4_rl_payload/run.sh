#!/bin/bash
export PX4_SIM_SPEED_FACTOR=1
DONT_RUN=1 make px4_sitl_default gazebo no_sim=1
source ~/Workspaces/auro_ws/devel/setup.bash
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 mavros_posix_sitl.launch vehicle:='iris_load_test' 
