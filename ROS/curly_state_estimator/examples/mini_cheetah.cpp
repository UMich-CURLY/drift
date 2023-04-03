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
using namespace state;

int main(int argc, char** argv) {
  /// TUTORIAL: Initialize ROS node
  ros::init(argc, argv, "robot_state_est");

  std::cout << "The subscriber is on!" << std::endl;

  /// TUTORIAL: Initialize ROS node handle. ROS handle handles the
  /// start/shutdown for us
  ros::NodeHandle nh;

  /// TUTORIAL: Create a ROS subscriber
  ros_wrapper::ROSSubscriber ros_sub(&nh);

  /// TUTORIAL: Add a subscriber for IMU data and get its queue and mutex
  std::cout << "Subscribing to imu channel..." << std::endl;
  auto qimu_and_mutex = ros_sub.AddIMUSubscriber("/Imu");
  auto qimu = qimu_and_mutex.first;
  auto qimu_mutex = qimu_and_mutex.second;

  /// TUTORIAL: Add a subscriber for legged kinematics data and get its queue
  std::cout << "Subscribing to joint_states and contact channel..."
            << std::endl;
  auto qkin_and_mutex
      = ros_sub.AddMiniCheetahKinematicsSubscriber("/Contacts", "/JointState");
  auto qkin = qkin_and_mutex.first;
  auto qkin_mutex = qkin_and_mutex.second;

  /// TUTORIAL: Start the subscriber thread
  ros_sub.StartSubscribingThread();

  /// TUTORIAL: Define some configurations for the state estimator
  inekf::ErrorType error_type = LeftInvariant;
  YAML::Node config_ = YAML::LoadFile(
      "config/filter/inekf/propagation/mini_cheetah_imu_propagation.yaml");
  bool enable_imu_bias_update
      = config_["settings"]["enable_imu_bias_update"].as<bool>();

  /// TUTORIAL: Create a state estimator
  StateEstimator state_estimator(error_type, enable_imu_bias_update);

  /// TUTORIAL: Add a propagation and correction(s) to the state estimator
  // Mini Cheetah's setting:
  state_estimator.add_imu_propagation(
      qimu, qimu_mutex,
      "config/filter/inekf/propagation/mini_cheetah_imu_propagation.yaml");
  state_estimator.add_legged_kinematics_correction(
      qkin, qkin_mutex,
      "config/filter/inekf/correction/"
      "mini_cheetah_legged_kinematics_correction.yaml");

  /// TUTORIAL: Get the robot state queue and mutex from the state estimator
  RobotStateQueuePtr robot_state_queue_ptr
      = state_estimator.get_robot_state_queue_ptr();
  std::shared_ptr<std::mutex> robot_state_queue_mutex_ptr
      = state_estimator.get_robot_state_queue_mutex_ptr();

  /// TUTORIAL: Create a ROS publisher and start the publishing thread
  ros_wrapper::ROSPublisher ros_pub(&nh, robot_state_queue_ptr,
                                    robot_state_queue_mutex_ptr);
  ros_pub.StartPublishingThread();

  /// TUTORIAL: Run the state estimator. Initialize the bias first, then
  /// initialize the state. After that, the state estimator will be enabled.
  /// The state estimator should be run in a loop. Users can use RVIZ to
  /// visualize the path. The path topic will be something like
  /// "/robot/*/path"
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