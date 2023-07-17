/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
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
#include <filesystem>
#include <iostream>

#include "communication/ros_publisher.h"
#include "communication/ros_subscriber.h"
#include "wheel_odometry/wheel_odometry.h"

using namespace std;
using namespace state;


int main(int argc, char** argv) {
  /// TUTORIAL: Initialize ROS node
  ros::init(argc, argv, "husky_wheel_odom");

  std::cout << "The subscriber is on!" << std::endl;

  /// TUTORIAL: Initialize ROS node handle. ROS handle handles the
  /// start/shutdown for us
  ros::NodeHandle nh;

  /// TUTORIAL: Create a ROS subscriber
  ros_wrapper::ROSSubscriber ros_sub(&nh);

  /// TUTORIAL: Load your yaml file
  // Find current path
  std::string file{__FILE__};
  std::string project_dir{file.substr(0, file.rfind("ROS/drift/baselines/"))};
  std::cout << "Project directory: " << project_dir << std::endl;

  std::string ros_config_file
      = project_dir
        + "/ROS/drift/config/baselines/husky_wheel_odometry/ros_comm.yaml";
  YAML::Node config = YAML::LoadFile(ros_config_file);
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
  /// TUTORIAL: Add a subscriber for velocity data and get its queue and mutex
  auto [qv, qv_mutex, qangv, qangv_mutex]
      = ros_sub.AddDifferentialDriveVelocitySubscriber(
          wheel_encoder_topic, wheel_radius, track_width);

  /// TUTORIAL: Start the subscriber thread
  ros_sub.StartSubscribingThread();

  baseline::WheelOdometry odom_estimator(
      qv, qv_mutex, qangv, qangv_mutex,
      project_dir + "/config/baselines/husky_wheel_odometry/config.yaml");


  /// TUTORIAL: Get the robot state queue and mutex from the state estimator
  // RobotStateQueuePtr robot_state_queue_ptr
  //     = inekf_estimator.get_robot_state_queue_ptr();
  // std::shared_ptr<std::mutex> robot_state_queue_mutex_ptr
  //     = inekf_estimator.get_robot_state_queue_mutex_ptr();

  // /// TUTORIAL: Create a ROS publisher and start the publishing thread
  // ros_wrapper::ROSPublisher ros_pub(
  //     &nh, robot_state_queue_ptr, robot_state_queue_mutex_ptr,
  //     ros_config_file);
  // ros_pub.StartPublishingThread();


  while (ros::ok()) {
    // Step behavior
    odom_estimator.RunOnce();
    ros::spinOnce();
  }

  return 0;
}