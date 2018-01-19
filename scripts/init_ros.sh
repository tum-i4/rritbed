#!/bin/bash

args=`getopt f: $*`

if (( $# == 1 )); then
    file_name=$1
else
	file_name="ros.launch"
fi

cd ~/thesis/ma-ros_git/catkin_ws
catkin_make
source devel/setup.bash

roslaunch ~/thesis/ma-ros_git/scripts/$file_name

cd ~/thesis/ma-ros_git/scripts
