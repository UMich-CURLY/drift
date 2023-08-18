/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   wheel_odometry.cpp
 *  @author Tzu-Yuan Lin
 *  @brief  Source file for the wheel odometry method
 *  @date   May 16, 2023
 **/

#include "wheel_odometry/wheel_odometry.h"


using namespace std;
using namespace math::lie_group;

namespace baseline {

WheelOdometry::WheelOdometry(VelocityQueuePtr vel_queue,
                             std::shared_ptr<std::mutex> vel_queue_mutex,
                             AngularVelocityQueuePtr ang_vel_queue,
                             std::shared_ptr<std::mutex> ang_vel_queue_mutex,
                             std::string config_file) {
  vel_queue_ptr_ = vel_queue;
  vel_queue_mutex_ptr_ = vel_queue_mutex;
  ang_vel_queue_ptr_ = ang_vel_queue;
  ang_vel_queue_mutex_ptr_ = ang_vel_queue_mutex;


  YAML::Node config = YAML::LoadFile(config_file);
  enable_pose_logger_ = config["logger"]["enable_pose_logger"].as<bool>();

  if (enable_pose_logger_) {
    pose_log_file_ = config["logger"]["pose_log_file"].as<std::string>();

    // Erase the ".txt" or ".csv" from the file name.
    // This is helpful to make new file names when InekfEstimator::clear() is
    // called
    std::string::size_type pos = pose_log_file_.find_last_of(".");
    if (pos != std::string::npos)
      pose_log_file_.erase(pos, pose_log_file_.length());

    // Define logger settings
    pose_log_rate_ = config["logger"]["pose_log_rate"]
                         ? config["logger"]["pose_log_rate"].as<double>()
                         : 10.0;
    outfile_.open(pose_log_file_ + ".txt");
    outfile_.precision(dbl::max_digits10);

    prev_t_ = 0;
  }
}

WheelOdometry::~WheelOdometry() {
  if (enable_pose_logger_) {
    outfile_.close();
  }
}

void WheelOdometry::RunOnce() {
  VelocityMeasurementPtr vel_measurement = nullptr;
  AngularVelocityMeasurementPtr ang_vel_measurement = nullptr;

  // Make sure we get both velocity and angular velocity measurements
  vel_queue_mutex_ptr_.get()->lock();
  ang_vel_queue_mutex_ptr_.get()->lock();
  if (vel_queue_ptr_->empty() || ang_vel_queue_ptr_->empty()) {
    vel_queue_mutex_ptr_.get()->unlock();
    ang_vel_queue_mutex_ptr_.get()->unlock();
    return;
  } else {
    vel_measurement = vel_queue_ptr_->front();
    vel_queue_ptr_->pop();
    vel_queue_mutex_ptr_.get()->unlock();
    ang_vel_measurement = ang_vel_queue_ptr_->front();
    ang_vel_queue_ptr_->pop();
    ang_vel_queue_mutex_ptr_.get()->unlock();
  }


  if (vel_measurement != nullptr && ang_vel_measurement != nullptr) {
    // Get the time stamp of the measurement
    double t = vel_measurement->get_time();

    if (prev_t_ == 0) {
      prev_t_ = t;
      state_.set_time(t);
      return;
    }

    // Get the velocity measurement
    Eigen::Vector3d vel = vel_measurement->get_velocity();

    // Get the angular velocity measurement
    Eigen::Vector3d ang_vel = ang_vel_measurement->get_angular_velocity();

    // Get the time difference between the current and previous measurement
    double dt = t - prev_t_;

    // Update the previous time stamp
    prev_t_ = t;

    Integrate(vel, ang_vel, dt);

    LogPose();
  }
}

void WheelOdometry::Integrate(Eigen::Vector3d v_mea, Eigen::Vector3d w,
                              double dt) {
  // Integrate the velocity and angular velocity
  Eigen::Matrix3d R = state_.get_rotation();
  Eigen::Vector3d v = state_.get_velocity();
  Eigen::Vector3d p = state_.get_position();

  Eigen::Vector3d phi = w * dt;
  Eigen::Matrix3d exp_omega = Gamma_SO3(phi, 0);

  Eigen::MatrixXd X = Eigen::MatrixXd::Ones(5, 5);
  X.block<3, 3>(0, 0) = R * exp_omega;
  X.block<3, 1>(0, 3) = R * v_mea;
  X.block<3, 1>(0, 4) = p + R * v_mea * dt;

  state_.set_X(X);
  state_.set_time(state_.get_time() + dt);
}

void WheelOdometry::LogPose() {
  double time_diff = state_.get_time() - last_pub_t_;
  if (enable_pose_logger_ && time_diff >= 1.0 / pose_log_rate_) {
    Eigen::Quaterniond state_quat(state_.get_world_rotation());
    outfile_ << state_.get_time() << " " << state_.get_world_position()(0)
             << " " << state_.get_world_position()(1) << " "
             << state_.get_world_position()(2) << " " << state_quat.x() << " "
             << state_quat.y() << " " << state_quat.z() << " " << state_quat.w()
             << std::endl
             << std::flush;
    last_pub_t_ = state_.get_time();
  }
}
}    // namespace baseline