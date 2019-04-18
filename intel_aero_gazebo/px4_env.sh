#!/bin/bash

# PX4
PX4_PATH=$HOME/Documents/PX4/Firmware
{
  source $PX4_PATH/Tools/setup_gazebo.bash $PX4_PATH $PX4_PATH/build/px4_sitl_default
} &> /dev/null # surpress output of setup_gazebo.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_PATH/Tools/sitl_gazebo
