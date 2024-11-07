#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, Point
from std_msgs.msg import Header
import time

def read_ground_truth_file(file_path):
    """ Reads the ground truth file and returns a list of odometry data """
    odom_data = []
    with open(file_path, 'r') as file:
        for line in file:
            if line.strip():  # Ignore empty lines
                parts = line.split()
                timestamp = float(parts[0])
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
                quat_x = float(parts[4])
                quat_y = float(parts[5])
                quat_z = float(parts[6])
                quat_w = float(parts[7])
                odom_data.append([timestamp, x, y, z, quat_x, quat_y, quat_z, quat_w])
    return odom_data

def publish_odometry(odom_data):
    """ Publishes odometry data from the file """
    pub = rospy.Publisher('/gt_odom', Odometry, queue_size=10)
    rospy.init_node('odom_publisher_node', anonymous=True)
    rate = rospy.Rate(60)  # 60 Hz (adjust as needed)

    for entry in odom_data:
        if rospy.is_shutdown():
            break

        # Extract timestamp and pose information from odom_data
        timestamp, x, y, z, quat_x, quat_y, quat_z, quat_w = entry

        # Convert the UNIX epoch timestamp to ROS Time
        ros_time = rospy.Time.from_sec(timestamp)

        # Create Odometry message
        odom_msg = Odometry()
        
        # Header
        odom_msg.header = Header()
        odom_msg.header.stamp = ros_time  # Use the timestamp from the file
        odom_msg.header.frame_id = "map"  # Frame of reference

        # Set position and orientation
        odom_msg.pose.pose = Pose()
        odom_msg.pose.pose.position = Point(x, y, z)
        odom_msg.pose.pose.orientation = Quaternion(quat_x, quat_y, quat_z, quat_w)

        # Publish the message
        pub.publish(odom_msg)
        rospy.loginfo("Published odom message at timestamp: %f", timestamp)

        # Sleep to simulate real-time publishing based on timestamp differences
        rate.sleep()

if __name__ == '__main__':
    try:
        # Path to the ground_truth.txt file
        file_path = 'ground_truth.txt'

        odom_data = read_ground_truth_file(file_path)
        
        # Only call the publishing function once and stop when data is exhausted
        publish_odometry(odom_data)

    except rospy.ROSInterruptException:
        pass

