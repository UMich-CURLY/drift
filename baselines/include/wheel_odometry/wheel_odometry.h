/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   wheel_odometry.h
 *  @author Tzu-Yuan LIn
 *  @brief  Header file for the wheel odometry method
 *  @date   July 14, 2023
 **/

#ifndef BASELINE_WHEEL_ODOMETRY_H
#define BASELINE_WHEEL_ODOMETRY_H

#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <queue>
#include <vector>

#include <Eigen/Dense>
#include "yaml-cpp/yaml.h"

#include "drift/measurement/angular_velocity.h"
#include "drift/measurement/measurement.h"
#include "drift/state/robot_state.h"
#include "drift/utils/type_def.h"

using namespace state;

namespace baseline {
class WheelOdometry {
 public:
  WheelOdometry(VelocityQueuePtr vel_queue,
                std::shared_ptr<std::mutex> vel_queue_mutex,
                AngularVelocityQueuePtr ang_vel_queue,
                std::shared_ptr<std::mutex> ang_vel_queue_mutex,
                std::string config_file);
  ~WheelOdometry();

  void RunOnce();

 private:
  void Integrate(Eigen::Vector3d vel, Eigen::Vector3d ang_vel, double dt);

  void LogPose();

  RobotState state_;

  double prev_t_ = 0.0;
  double last_pub_t_ = 0.0;

  VelocityQueuePtr vel_queue_ptr_;
  AngularVelocityQueuePtr ang_vel_queue_ptr_;
  std::shared_ptr<std::mutex> vel_queue_mutex_ptr_;
  std::shared_ptr<std::mutex> ang_vel_queue_mutex_ptr_;

  bool enable_pose_logger_;
  std::string pose_log_file_;
  double pose_log_rate_;
  std::ofstream outfile_;
};
}    // namespace baseline

#endif    // BASELINE_WHEEL_ODOMETRY_H