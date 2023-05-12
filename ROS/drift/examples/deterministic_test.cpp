/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   fetch.cpp
 *  @author Tingjun Li
 *  @brief  Test file for Fetch robot (IMU Propagation + Velocity Correction)
 *  @date   March 20, 2023
 **/

#include <ros/ros.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

#include "communication/ros_publisher.h"
#include "communication/ros_subscriber.h"
#include "drift/estimator/inekf_estimator.h"

using namespace std;
using namespace state;
using namespace estimator;


int main(int argc, char** argv) {
  /// TUTORIAL: Initialize ROS node
  ros::init(argc, argv, "deterministic_test");

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
      = project_dir + "/ROS/drift/config/deterministic_test/ros_comm.yaml";
  YAML::Node config = YAML::LoadFile(config_file);
  std::string imu_topic = config["subscribers"]["imu_topic"].as<std::string>();
  std::string wheel_encoder_topic
      = config["subscribers"]["wheel_encoder_topic"].as<std::string>();
  double wheel_radius = config["subscribers"]["wheel_radius"].as<double>();
  double track_width = config["subscribers"]["track_width"].as<double>();
  int sleep_time = config["subscribers"]["sleep_time"].as<int>();


  /// REMARK: To use Fetch's IMU data, please use the following line
  /// ("AddFetchIMUSubscriber") instead of the "AddIMUSubscriber"
  // auto qimu_and_mutex
  // = ros_sub.AddFetchIMUSubscriber(imu_topic, "/imu1/gyro_offset");

  /// TUTORIAL: Add a subscriber for IMU data and get its queue and mutex
  IMUQueuePtr qimu(new IMUQueue);
  std::shared_ptr<std::mutex> qimu_mutex(new std::mutex);

  /// TUTORIAL: Add a subscriber for velocity data and get its queue and mutex
  VelocityQueuePtr qv(new VelocityQueue);
  std::shared_ptr<std::mutex> qv_mutex(new std::mutex);
  AngularVelocityQueuePtr qangv(new AngularVelocityQueue);
  std::shared_ptr<std::mutex> qangv_mutex(new std::mutex);

  /// TUTORIAL: Define some configurations for the state estimator
  inekf::ErrorType error_type = LeftInvariant;

  /// TUTORIAL: Create a state estimator
  InekfEstimator inekf_estimator(
      error_type,
      project_dir + "/config/deterministic_test/inekf_estimator.yaml");

  /// TUTORIAL: Add a propagation and correction(s) to the state estimator:
  auto [q_filtered_imu, q_filtered_imu_mutex] =
  inekf_estimator.add_imu_ang_vel_ekf(
      qimu, qimu_mutex, qangv, qangv_mutex,
      project_dir + "/config/deterministic_test/"
      "imu_filter.yaml");    // Fetch's setting

  // Insert the IMU data
  inekf_estimator.add_imu_propagation(
      q_filtered_imu, q_filtered_imu_mutex,
      project_dir + "/config/deterministic_test/imu_propagation.yaml");

  // inekf_estimator.add_imu_propagation(
  //     qimu, qimu_mutex,
  //     project_dir + "/config/deterministic_test/imu_propagation.yaml");
  inekf_estimator.add_velocity_correction(
      qv, qv_mutex,
      project_dir + "/config/deterministic_test/velocity_correction.yaml");

  /// TUTORIAL: Get the robot state queue and mutex from the state estimator
  RobotStateQueuePtr robot_state_queue_ptr
      = inekf_estimator.get_robot_state_queue_ptr();
  std::shared_ptr<std::mutex> robot_state_queue_mutex_ptr
      = inekf_estimator.get_robot_state_queue_mutex_ptr();

  /// TUTORIAL: Create a ROS publisher and start the publishing thread
  ros_wrapper::ROSPublisher ros_pub(&nh, robot_state_queue_ptr,
                                    robot_state_queue_mutex_ptr, config_file);
  ros_pub.StartPublishingThread();

  /// TUTORIAL: Load the data file
  std::string data_file_name
      = "/home/neofelis/Downloads/synced_js_imu_bias.csv";
  std::fstream data_file;
  data_file.open(data_file_name, std::ios::in);
  if (!data_file.is_open()) {
    std::cout << "Cannot open data file!" << std::endl;
    return 0;
  }

  std::vector<int> seq;
  std::vector<double> timestamps;
  std::vector<double> vel_left, vel_right;
  std::vector<double> ori_x, ori_y, ori_z, ori_w, ang_vel_x, ang_vel_y,
      ang_vel_z, lin_acc_x, lin_acc_y, lin_acc_z;
  std::vector<double> gyro_bias_x, gyro_bias_y, gyro_bias_z;

  // Read data from the file
  std::vector<double> row;
  string line, value, temp;

  // Skip the first line
  getline(data_file, line);

  while (getline(data_file, line)) {
    row.clear();
    std::stringstream s(line);
    while (getline(s, value, ',')) {
      if (!value.empty()) {
        row.push_back(std::stod(value));
      }
    }

    seq.push_back(row[1]);
    timestamps.push_back(row[2] + row[3] / 1e9);
    vel_left.push_back(row[6]);
    vel_right.push_back(row[7]);
    ori_x.push_back(row[8]);
    ori_y.push_back(row[9]);
    ori_z.push_back(row[10]);
    ori_w.push_back(row[11]);
    ang_vel_x.push_back(row[12]);
    ang_vel_y.push_back(row[13]);
    ang_vel_z.push_back(row[14]);
    lin_acc_x.push_back(row[15]);
    lin_acc_y.push_back(row[16]);
    lin_acc_z.push_back(row[17]);
    gyro_bias_x.push_back(row[18]);
    gyro_bias_y.push_back(row[19]);
    gyro_bias_z.push_back(row[20]);
  }

  int max_size = timestamps.size() * 2;    // make sure all the data are used
  int i = 0;

  /// TUTORIAL: Run the state estimator. Initialize the bias first, then
  /// initialize the state. After that, the state estimator will be enabled.
  /// The state estimator should be run in a loop. Users can use RVIZ to
  /// visualize the path. The path topic will be something like
  /// "/robot/*/path"

  while (ros::ok() && i < max_size) {
    // Load the data to measurement
    if (i < timestamps.size()) {
      // Timestamp:
      double timestamp = timestamps[i];
      // IMU measurement:
      std::shared_ptr<ImuMeasurement<double>> imu_measurement_ptr(
          new ImuMeasurement<double>);
      imu_measurement_ptr->set_header(seq[i], timestamp, "");
      imu_measurement_ptr->set_ang_vel(ang_vel_x[i], ang_vel_y[i],
                                       ang_vel_z[i]);
      imu_measurement_ptr->set_lin_acc(lin_acc_x[i], lin_acc_y[i],
                                       lin_acc_z[i]);

      // Velocity measurement:
      std::shared_ptr<VelocityMeasurement<double>> vel_measurement(
          new VelocityMeasurement<double>);
      std::shared_ptr<AngularVelocityMeasurement<double>> ang_vel_measurement(
          new AngularVelocityMeasurement<double>);
      vel_measurement->set_header(seq[i], timestamp + 0.0001, "");
      ang_vel_measurement->set_header(seq[i], timestamp + 0.0001, "");
      double vr = vel_right[i] * wheel_radius;
      double vl = vel_left[i] * wheel_radius;
      double vx = (vr + vl) / 2;
      double omega_z = (vr - vl) / track_width;
      vel_measurement->set_velocity(vx, 0, 0);
      ang_vel_measurement->set_ang_vel(0, 0, omega_z);

      // Insert the measurement to the queue
      qimu_mutex->lock();
      qimu->push(imu_measurement_ptr);
      qimu_mutex->unlock();

      qv_mutex->lock();
      qv->push(vel_measurement);
      qv_mutex->unlock();

      qangv_mutex->lock();
      qangv->push(ang_vel_measurement);
      qangv_mutex->unlock();
      std::this_thread::sleep_for(std::chrono::microseconds(sleep_time));
    }

    std::cout << "i: " << i << std::endl;
    i++;

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

    qv_mutex->lock();
    if (qv->empty() && i >= timestamps.size()) {
      qv_mutex->unlock();
      break;
    }
    qv_mutex->unlock();
  }

  return 0;
}