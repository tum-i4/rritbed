#!/bin/bash

cd ~/thesis/ma-ros_git/catkin_ws
catkin_make
source devel/setup.bash

roslaunch ~/thesis/ma-ros_git/catkin_ws/src/turtlesim_expl/src/ros.launch

cd ~/thesis/ma-ros_git
