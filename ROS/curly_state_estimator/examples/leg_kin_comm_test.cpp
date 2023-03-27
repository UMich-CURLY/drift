/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   leg_kin_comm_test.cpp
 *  @author Tingjun Li
 *  @brief  Test file for Mini-Cheetah robot (IMU propagation + Legged
 *Kinematics Correction)
 *  @date   March 20, 2023
 **/

#include <ros/ros.h>
#include <iostream>

#include "communication/ros_publisher.h"
#include "communication/ros_subscriber.h"
#include "state_estimator.h"
using namespace std;


int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_state_est");

  std::cout << "The subscriber is on!" << std::endl;

  // ros handle handles the start/shutdown for us
  ros::NodeHandle nh;

  // Subscriber:
  ros_wrapper::ROSSubscriber ros_sub(&nh);
  std::cout << "Subscribing to imu channel..." << std::endl;
  auto qimu_and_mutex = ros_sub.AddIMUSubscriber("/Imu");
  auto qimu = qimu_and_mutex.first;
  auto qimu_mutex = qimu_and_mutex.second;

  std::cout << "Subscribing to joint_states and contact channel..."
            << std::endl;
  auto qkin_and_mutex
      = ros_sub.AddMiniCheetahKinematicsSubscriber("/Contacts", "/JointState");
  auto qkin = qkin_and_mutex.first;
  auto qkin_mutex = qkin_and_mutex.second;

  ros_sub.StartSubscribingThread();

  inekf::ErrorType error_type = LeftInvariant;
  YAML::Node config_ = YAML::LoadFile(
      "config/filter/inekf/propagation/mini_cheetah_imu_propagation.yaml");
  bool enable_imu_bias_update
      = config_["settings"]["enable_imu_bias_update"].as<bool>();
  StateEstimator state_estimator(error_type, enable_imu_bias_update);

  // Publisher:
  // Mini Cheetah's setting:
  state_estimator.add_imu_propagation(
      qimu, qimu_mutex,
      "config/filter/inekf/propagation/mini_cheetah_imu_propagation.yaml");
  state_estimator.add_legged_kinematics_correction(
      qkin, qkin_mutex,
      "config/filter/inekf/correction/"
      "mini_cheetah_legged_kinematics_correction.yaml");
  RobotStateQueuePtr robot_state_queue_ptr
      = state_estimator.get_robot_state_queue_ptr();

  std::shared_ptr<std::mutex> robot_state_queue_mutex_ptr
      = state_estimator.get_robot_state_queue_mutex_ptr();
  ros_wrapper::ROSPublisher ros_pub(&nh, robot_state_queue_ptr,
                                    robot_state_queue_mutex_ptr);
  ros_pub.StartPublishingThread();

  // Start running
  while (ros::ok()) {
    // Step behavior
    if (state_estimator.is_enabled()) {
      state_estimator.RunOnce();
    } else {
      if (state_estimator.BiasInitialized()) {
        state_estimator.InitState();
      } else {
        state_estimator.InitBias();
      }
    }
    ros::spinOnce();
  }
  return 0;
}