/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   ros_subsriber.h
 *  @author Justin Lin
 *  @brief  Header file for ROS subscriber class
 *  @date   December 6, 2022
 **/

#ifndef ROS_COMMUNICATION_ROS_SUBSCRIBER_H
#define ROS_COMMUNICATION_ROS_SUBSCRIBER_H

#include <string>
#include <thread>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

namespace ros_wrapper {

class ROSSubscriber {
 public:
  ROSSubscriber(ros::NodeHandle* nh);
  ~ROSSubscriber();

  void add_imu_subscriber(const std::string topic_name);

 private:
  void imu_call_back(const sensor_msgs::Imu& imu_msg);

  void ros_spin();

  ros::NodeHandle* nh_;
  std::vector<ros::Subscriber> subscriber_list_;

  std::thread subscribing_thread_;
};


}    // namespace ros_wrapper

#endif