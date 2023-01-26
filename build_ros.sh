#!/bin/bash
#
# Gencpp
# This files generates the necessarcy C++ files for your ROS msg/srv.
# This file will search in /msg and /srv and generate header files in /include/msg or /include/srv respectively. 
# Useage: ./gencpp.sh [namespace]
echo "Building custom_sensor_msgs"

MSG_NAMESPACE=custom_sensor_msgs
MSG_PATH=./ROS/curly_state_estimator/msg
MSG_HEADER_OUTPUT_PATH=./ROS/curly_state_estimator/include/$MSG_NAMESPACE/

for file in $MSG_PATH/*
do
	if [[ -a $file ]]
	then
		rosrun gencpp gen_cpp.py -p $MSG_NAMESPACE -o $MSG_HEADER_OUTPUT_PATH -e /opt/ros/noetic/share/gencpp "$file" -I std_msgs:/opt/ros/noetic/share/std_msgs/msg -I $MSG_NAMESPACE:$MSG_PATH
	fi
done


echo "Building ROS nodes"

cd ROS/curly_state_estimator
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release -Wno-dev 
make -j6