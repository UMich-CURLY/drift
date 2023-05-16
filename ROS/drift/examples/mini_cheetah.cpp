/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   mini_cheetah.cpp
 *  @author Tingjun Li
 *  @brief  Test file for Mini-Cheetah robot (IMU propagation + Legged
 *Kinematics Correction)
 *  @date   March 20, 2023
 **/

#include <ros/ros.h>
#include <iostream>

#include "communication/ros_publisher.h"
#include "communication/ros_subscriber.h"
#include "drift/estimator/inekf_estimator.h"

using namespace std;
using namespace state;
using namespace estimator;

int main(int argc, char** argv) {
  /// TUTORIAL: Initialize ROS node
  ros::init(argc, argv, "mini_cheetah");

  std::cout << "The subscriber is on!" << std::endl;

  /// TUTORIAL: Initialize ROS node handle. ROS handle handles the
  /// start/shutdown for us
  ros::NodeHandle nh;

  /// TUTORIAL: Create a ROS subscriber
  ros_wrapper::ROSSubscriber ros_sub(&nh);

  /// TUTORIAL: Load your yaml file
  // Find project path
  std::string file{__FILE__};
  std::string project_dir{file.substr(0, file.rfind("ROS/drift/examples/"))};
  std::cout << "Project directory: " << project_dir << std::endl;

  std::string config_file
      = project_dir + "/ROS/drift/config/mini_cheetah/ros_comm.yaml";
  YAML::Node config = YAML::LoadFile(config_file);
  std::string imu_topic = config["subscribers"]["imu_topic"].as<std::string>();
  std::string joint_encoder_topic
      = config["subscribers"]["joint_encoder_topic"].as<std::string>();
  std::string contact_topic
      = config["subscribers"]["contact_topic"].as<std::string>();

  /// TUTORIAL: Add a subscriber for IMU data and get its queue and mutex
  std::cout << "Subscribing to imu channel..." << std::endl;
  auto qimu_and_mutex = ros_sub.AddIMUSubscriber(imu_topic);
  auto qimu = qimu_and_mutex.first;
  auto qimu_mutex = qimu_and_mutex.second;

  /// TUTORIAL: Add a subscriber for legged kinematics data and get its queue
  std::cout << "Subscribing to joint_states and contact channel..."
            << std::endl;
  auto qkin_and_mutex = ros_sub.AddMiniCheetahKinematicsSubscriber(
      contact_topic, joint_encoder_topic);
  auto qkin = qkin_and_mutex.first;
  auto qkin_mutex = qkin_and_mutex.second;

  /// TUTORIAL: Start the subscriber thread
  ros_sub.StartSubscribingThread();

  /// TUTORIAL: Define some configurations for the state estimator
  inekf::ErrorType error_type = LeftInvariant;

  /// TUTORIAL: Create a state estimator
  InekfEstimator inekf_estimator(
      error_type, project_dir + "/config/mini_cheetah/inekf_estimator.yaml");

  /// TUTORIAL: Add a propagation and correction(s) to the state estimator
  // Mini Cheetah's setting:
  inekf_estimator.add_imu_propagation(
      qimu, qimu_mutex,
      project_dir + "/config/mini_cheetah/imu_propagation.yaml");
  inekf_estimator.add_legged_kinematics_correction(
      qkin, qkin_mutex,
      project_dir + "/config/mini_cheetah/legged_kinematics_correction.yaml");

  /// TUTORIAL: Get the robot state queue and mutex from the state estimator
  RobotStateQueuePtr robot_state_queue_ptr
      = inekf_estimator.get_robot_state_queue_ptr();
  std::shared_ptr<std::mutex> robot_state_queue_mutex_ptr
      = inekf_estimator.get_robot_state_queue_mutex_ptr();

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