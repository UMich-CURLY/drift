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
class ROSSubscriber {
 public:
  ROSSubscriber(ros::NodeHandle* nh);
  ~ROSSubscriber();

  IMUQueuePtr add_imu_subscriber(const std::string topic_name);
  VelocityQueuePtr add_velocity_subscriber(const std::string topic_name);
  VelocityQueuePtr add_differential_drive_velocity_subscriber(
      const std::string topic_name);
  void start_subscribing_thread();

 private:
  void imu_call_back(const boost::shared_ptr<const sensor_msgs::Imu>& imu_msg,
                     const std::shared_ptr<std::mutex>& mutex,
                     IMUQueuePtr& imu_queue);
  void velocity_call_back(
      const boost::shared_ptr<const geometry_msgs::Twist>& vel_msg,
      const std::shared_ptr<std::mutex>& mutex, VelocityQueuePtr& vel_queue);
  void differential_encoder2velocity_call_back(
      const boost::shared_ptr<const sensor_msgs::JointState>& encoder_msg,
      const std::shared_ptr<std::mutex>& mutex, VelocityQueuePtr& vel_queue);
  void ros_spin();

  ros::NodeHandle* nh_;
  std::vector<ros::Subscriber> subscriber_list_;

  // measurement queue list
  std::vector<IMUQueuePtr> imu_queue_list_;
  std::vector<VelocityQueuePtr> vel_queue_list_;
  std::vector<std::shared_ptr<std::mutex>> mutex_list_;

  bool thread_started_;
  std::thread subscribing_thread_;
};


}    // namespace ros_wrapper

#endif