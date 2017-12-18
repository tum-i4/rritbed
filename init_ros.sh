#!/bin/bash

gnome-terminal -e roscore
sleep 1

gnome-terminal -e rosrun turtlesim turtlesim_node __name:=ecu1
gnome-terminal -e rosrun turtlesim turtlesim_node __name:=ecu2
sleep 1

cd ~/thesis/ma-ros_git/catkin_ws
catkin_make
source devel/setup.bash

cd ~/thesis/ma-ros_git/catkin_ws/src/turtlesim_expl/src
rosrun turtlesim_expl basic_mover.py

cd ~/thesis/ma-ros_git
