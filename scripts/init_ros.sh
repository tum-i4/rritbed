#!/bin/bash

args=`getopt f: $*`

base_path="~/thesis/ma-ros_git/scripts"

if (( $# == 2 )); then
    file_name="$2.launch"
else
	file_name="ros.launch"
fi

file_path="$base_path/$file_name"

cd ~/thesis/ma-ros_git/catkin_ws
catkin_make
source devel/setup.bash

roslaunch $file_path

cd ~/thesis/ma-ros_git/scripts
