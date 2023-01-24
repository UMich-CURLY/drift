#!/bin/bash
#
# Gencpp
# This files generates the necessarcy C++ files for your ROS msg/srv.
# This file will search in /msg and /srv and generate header files in /include/msg or /include/srv respectively. 
# Useage: ./gencpp.sh [namespace]
if [[ -z $1 ]]
then
	echo "You have to specify a namespace"
	exit 1
fi

for file in ./srv/*
do
	if [[ -a $file ]]
	then
		rosrun gencpp gen_cpp.py -p "$1" -o ./include/srv/ -e /opt/ros/noetic/share/gencpp "$file"
	fi
done

for file in ./msg/*
do
	if [[ -a $file ]]
	then
		rosrun gencpp gen_cpp.py -p "$1" -o ./include/custom_sensor_msgs/ -e /opt/ros/noetic/share/gencpp "$file" -I std_msgs:/opt/ros/noetic/share/std_msgs/msg -I custom_sensor_msgs:./msg
	fi
done
