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

#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "boost/bind.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

#include "measurement/imu.h"

typedef std::queue<std::shared_ptr<ImuMeasurement<double>>> IMUQueue;
typedef std::shared_ptr<IMUQueue> IMUQueuePtr;

namespace ros_wrapper {
class ROSSubscriber {
 public:
  ROSSubscriber(ros::NodeHandle* nh);
  ~ROSSubscriber();

  IMUQueuePtr add_imu_subscriber(const std::string topic_name);

 private:
  void imu_call_back(const boost::shared_ptr<const sensor_msgs::Imu>& imu_msg,
                     IMUQueuePtr& imu_queue, std::shared_ptr<std::mutex> mutex);
  void ros_spin();

  ros::NodeHandle* nh_;
  std::vector<ros::Subscriber> subscriber_list_;

  // measurement queue list
  std::vector<IMUQueuePtr> imu_queue_list_;
  std::vector<std::shared_ptr<std::mutex>> mutex_list_;

  std::thread subscribing_thread_;

  int test_idx = 1;
};


}    // namespace ros_wrapper

#endif