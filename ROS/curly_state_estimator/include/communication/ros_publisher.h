/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   ros_publisher.h
 *  @author Tingjun Li
 *  @brief  Header file for ROS subscriber class
 *  @date   December 20, 2022
 **/

#ifndef ROS_COMMUNICATION_ROS_PUBLISHER_H
#define ROS_COMMUNICATION_ROS_PUBLISHER_H

#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "boost/bind.hpp"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"

#include "measurement/imu.h"
#include "measurement/velocity.h"

typedef std::queue<std::shared_ptr<ImuMeasurement<double>>> IMUQueue;
typedef std::shared_ptr<IMUQueue> IMUQueuePtr;
typedef std::queue<std::shared_ptr<VelocityMeasurement<double>>> VelocityQueue;
typedef std::shared_ptr<VelocityQueue> VelocityQueuePtr;

namespace ros_wrapper {
class ROSPublisher {
 public:
  ROSPublisher(ros::NodeHandle* nh);
  ~ROSPublisher();


 private:
  ros::NodeHandle* nh_;

  bool thread_started_;

  ros::Publisher pose_pub_;
  std::string pose_frame_;
  uint32_t pose_seq_ = 0;
  double pose_publish_rate_;
  std::thread pose_publishing_thread_;

  ros::Publisher path_pub_;
  uint32_t path_seq_ = 0;
  double path_publish_rate_;
  std::thread path_publishing_thread_;

  int pose_skip_;

  std::array<float, 3> first_pose_;
  std::vector<geometry_msgs::PoseStamped> poses_;
  std::mutex poses_mutex_;

  void pathPublishingThread();
  void pathPublish();

  void posePublishingThread();
  void posePublish();
};


}    // namespace ros_wrapper

#endif