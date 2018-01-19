#!/bin/bash

args=`getopt f: $*`

base_path="~/thesis/ma-ros_git/scripts"

if (( $# == 1 )); then
    file_name=$1
else
	file_name="ros.launch"
fi

file_path="$base_path/$file_name"

cd ~/thesis/ma-ros_git/catkin_ws
catkin_make
source devel/setup.bash

roslaunch $file_path

cd ~/thesis/ma-ros_git/scripts
