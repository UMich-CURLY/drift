#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import numpy as np

# Initialize the first position
initial_position = None

# Define your transformation matrix here
def get_transformation():
    # Translation is always set to [0, 0, 0]
    translation = np.array([0.0, 0.0, 0.0])  # No translation
    rotation = tf.transformations.quaternion_matrix([0, 0, 0, 1])  # Identity quaternion (no rotation)
    transformation = rotation
    transformation[:3, 3] = translation  # Keep translation part as [0, 0, 0]
    return transformation

def apply_transformation(pose, transformation, initial_position):
    # Subtract the initial position to make the first position [0, 0, 0]
    relative_position = np.array([pose.position.x - initial_position[0],
                                  pose.position.y - initial_position[1],
                                  pose.position.z - initial_position[2], 1.0])
    
    quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    rotation_matrix = tf.transformations.quaternion_matrix(quaternion)
    current_pose_matrix = rotation_matrix
    current_pose_matrix[:3, 3] = relative_position[:3]

    # Apply the transformation (which has translation [0, 0, 0])
    transformed_pose_matrix = np.dot(transformation, current_pose_matrix)

    # Convert back to geometry_msgs/Pose
    transformed_pose = PoseStamped()
    transformed_pose.pose.position.x = transformed_pose_matrix[0, 3]
    transformed_pose.pose.position.y = transformed_pose_matrix[1, 3]
    transformed_pose.pose.position.z = transformed_pose_matrix[2, 3]

    transformed_quaternion = tf.transformations.quaternion_from_matrix(transformed_pose_matrix)
    transformed_pose.pose.orientation.x = transformed_quaternion[0]
    transformed_pose.pose.orientation.y = transformed_quaternion[1]
    transformed_pose.pose.orientation.z = transformed_quaternion[2]
    transformed_pose.pose.orientation.w = transformed_quaternion[3]

    return transformed_pose

def odom_callback(odom_msg, args):
    global initial_position
    path_pub, path_msg = args

    # Set the initial position if it's the first odometry message
    if initial_position is None:
        initial_position = [odom_msg.pose.pose.position.x, 
                            odom_msg.pose.pose.position.y, 
                            odom_msg.pose.pose.position.z]

    transformation = get_transformation()

    # Apply transformation to the current odometry pose relative to the initial position
    transformed_pose = apply_transformation(odom_msg.pose.pose, transformation, initial_position)
    transformed_pose.header = odom_msg.header

    # Append to Path message
    path_msg.poses.append(transformed_pose)

    # Publish the path
    path_pub.publish(path_msg)

def main():
    rospy.init_node('odom_to_path_node')

    # Publisher
    path_pub = rospy.Publisher('/transformed_path', Path, queue_size=10)
    
    path_msg = Path()
    path_msg.header.frame_id = 'map'

    # Subscriber to the /odom topic
    rospy.Subscriber('/gt_odom', Odometry, odom_callback, (path_pub, path_msg))

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

