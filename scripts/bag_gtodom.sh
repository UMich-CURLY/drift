#!/bin/bash
python3 odom_to_path.py &
#rosbag play output.bag
#rosbag play 1_rectangle.bag --start 15.44854 &
rosbag play 1_rectangle.bag &
python3 odom_pub.py &
wait
