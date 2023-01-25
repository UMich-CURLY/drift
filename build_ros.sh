#!/bin/bash
#
# Gencpp
# This files generates the necessarcy C++ files for your ROS msg/srv.
# This file will search in /msg and /srv and generate header files in /include/msg or /include/srv respectively. 
# Useage: ./gencpp.sh [namespace]
echo "Building custom_sensor_msgs"

for file in ./ROS/curly_state_estimator/srv/*
do
	if [[ -a $file ]]
	then
		rosrun gencpp gen_cpp.py -p custom_sensor_msgs -o ./include/srv/ -e /opt/ros/noetic/share/gencpp "$file"
	fi
done

for file in ./ROS/curly_state_estimator/msg/*
do
	if [[ -a $file ]]
	then
		rosrun gencpp gen_cpp.py -p custom_sensor_msgs -o ./include/custom_sensor_msgs/ -e /opt/ros/noetic/share/gencpp "$file" -I std_msgs:/opt/ros/noetic/share/std_msgs/msg -I custom_sensor_msgs:./msg
	fi
done


echo "Building ROS nodes"

cd ROS/curly_state_estimator
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release -Wno-dev 
make -j6