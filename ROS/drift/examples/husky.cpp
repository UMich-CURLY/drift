/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   husky.cpp
 *  @author Tingjun Li
 *  @brief  Test file for Clearpath Husky robot setup (IMU Propagation +
 *  Velocity Correction)
 *  @date   March 20, 2023
 **/

#include <ros/ros.h>
#include <iostream>

#include "communication/ros_publisher.h"
#include "communication/ros_subscriber.h"
#include "estimator/inekf_estimator.h"

using namespace std;
using namespace state;
using namespace estimator;


int main(int argc, char** argv) {
  /// TUTORIAL: Initialize ROS node
  ros::init(argc, argv, "robot_state_est");

  std::cout << "The subscriber is on!" << std::endl;

  /// TUTORIAL: Initialize ROS node handle. ROS handle handles the
  /// start/shutdown for us
  ros::NodeHandle nh;

  /// TUTORIAL: Create a ROS subscriber
  ros_wrapper::ROSSubscriber ros_sub(&nh);

  /// TUTORIAL: Load your yaml file
  std::string config_file
      = "/home/neofelis/Code/drift/ROS/drift/config/husky/ros_comm.yaml";
  YAML::Node config = YAML::LoadFile(config_file);
  std::string imu_topic = config["subscribers"]["imu_topic"].as<std::string>();
  std::string wheel_encoder_topic
      = config["subscribers"]["wheel_encoder_topic"].as<std::string>();
  double wheel_radius = config["subscribers"]["wheel_radius"].as<double>();
  double track_width = config["subscribers"]["track_width"].as<double>();

  /// TUTORIAL: Add a subscriber for IMU data and get its queue and mutex
  auto qimu_and_mutex = ros_sub.AddIMUSubscriber(imu_topic);
  auto qimu = qimu_and_mutex.first;
  auto qimu_mutex = qimu_and_mutex.second;

  /// TUTORIAL: Add a subscriber for velocity data and get its queue and mutex
  auto qv_and_mutex = ros_sub.AddDifferentialDriveVelocitySubscriber(
      wheel_encoder_topic, wheel_radius, track_width);
  auto qv = qv_and_mutex.first;
  auto qv_mutex = qv_and_mutex.second;

  /// TUTORIAL: Start the subscriber thread
  ros_sub.StartSubscribingThread();

  /// TUTORIAL: Define some configurations for the state estimator
  inekf::ErrorType error_type = LeftInvariant;

  /// TUTORIAL: Create a state estimator
  InekfEstimator inekf_estimator(error_type,
                                 "config/husky/inekf_estimator.yaml");

  /// TUTORIAL: Add a propagation and correction(s) methods to the state
  /// estimator. Here is an example of IMU propagation and velocity correction
  /// for Husky robot
  inekf_estimator.add_imu_propagation(qimu, qimu_mutex,
                                      "config/husky/imu_propagation.yaml");
  inekf_estimator.add_velocity_correction(
      qv, qv_mutex, "config/husky/velocity_correction.yaml");


  /// TUTORIAL: Get the robot state queue and mutex from the state estimator
  RobotStateQueuePtr robot_state_queue_ptr
      = inekf_estimator.get_robot_state_queue_ptr();
  std::shared_ptr<std::mutex> robot_state_queue_mutex_ptr
      = inekf_estimator.get_robot_state_queue_mutex_ptr();

  /// TUTORIAL: Create a ROS publisher and start the publishing thread
  ros_wrapper::ROSPublisher ros_pub(&nh, robot_state_queue_ptr,
                                    robot_state_queue_mutex_ptr, config_file);
  ros_pub.StartPublishingThread();

  /// TUTORIAL: Run the state estimator. Initialize the bias first, then
  /// initialize the state. After that, the state estimator will be enabled.
  /// The state estimator should be run in a loop. Users can use RVIZ to
  /// visualize the path. The path topic will be something like
  /// "/robot/*/path"
  while (ros::ok()) {
    // Step behavior
    if (inekf_estimator.is_enabled()) {
      inekf_estimator.RunOnce();
    } else {
      if (inekf_estimator.BiasInitialized()) {
        inekf_estimator.InitState();
      } else {
        inekf_estimator.InitBias();
      }
    }
    ros::spinOnce();
  }

  return 0;
}