/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   inekf_estimator.cpp
 *  @author Tzu-Yuan Lin, Tingjun Li
 *  @brief  Source file for state estimator class
 *  @date   December 1, 2022
 **/

#include "estimator/inekf_estimator.h"

namespace estimator {
InekfEstimator::InekfEstimator()
    : robot_state_queue_ptr_(new RobotStateQueue),
      robot_state_queue_mutex_ptr_(new std::mutex) {}

InekfEstimator::InekfEstimator(bool enable_imu_bias_update)
    : enable_imu_bias_update_(enable_imu_bias_update),
      robot_state_queue_ptr_(new RobotStateQueue),
      robot_state_queue_mutex_ptr_(new std::mutex) {}

InekfEstimator::InekfEstimator(ErrorType error_type,
                               bool enable_imu_bias_update)
    : error_type_(error_type),
      enable_imu_bias_update_(enable_imu_bias_update),
      robot_state_queue_ptr_(new RobotStateQueue),
      robot_state_queue_mutex_ptr_(new std::mutex) {}

void InekfEstimator::RunOnce() {
  // Propagate
  new_pose_ready_ = propagation_.get()->Propagate(state_);

  // Correct
  for (auto& correction : corrections_) {
    if (correction.get()->Correct(state_)) {
      new_pose_ready_ = true;
    }
  }

  // Publish when new information is added
  if (new_pose_ready_) {
    robot_state_queue_mutex_ptr_.get()->lock();
    robot_state_queue_ptr_.get()->push(std::make_shared<RobotState>(state_));
    robot_state_queue_mutex_ptr_.get()->unlock();
  }
  new_pose_ready_ = false;
}

RobotStateQueuePtr InekfEstimator::get_robot_state_queue_ptr() {
  return robot_state_queue_ptr_;
}

std::shared_ptr<std::mutex> InekfEstimator::get_robot_state_queue_mutex_ptr() {
  return robot_state_queue_mutex_ptr_;
}

void InekfEstimator::set_state(const RobotState& state) { state_ = state; }

const RobotState InekfEstimator::get_state() const { return state_; }

void InekfEstimator::add_imu_propagation(
    IMUQueuePtr buffer_ptr, std::shared_ptr<std::mutex> buffer_mutex_ptr,
    const std::string& yaml_filepath) {
  propagation_ = std::make_shared<ImuPropagation>(
      buffer_ptr, buffer_mutex_ptr, error_type_, enable_imu_bias_update_,
      yaml_filepath);
}

void InekfEstimator::add_legged_kinematics_correction(
    LeggedKinematicsQueuePtr buffer_ptr,
    std::shared_ptr<std::mutex> buffer_mutex_ptr,
    const std::string& yaml_filepath) {
  std::shared_ptr<Correction> correction
      = std::make_shared<LeggedKinematicsCorrection>(
          buffer_ptr, buffer_mutex_ptr, error_type_, enable_imu_bias_update_,
          yaml_filepath);
  corrections_.push_back(correction);
}

void InekfEstimator::add_velocity_correction(
    VelocityQueuePtr buffer_ptr, std::shared_ptr<std::mutex> buffer_mutex_ptr,
    const std::string& yaml_filepath) {
  std::shared_ptr<Correction> correction = std::make_shared<VelocityCorrection>(
      buffer_ptr, buffer_mutex_ptr, error_type_, enable_imu_bias_update_,
      yaml_filepath);
  corrections_.push_back(correction);
}

const bool InekfEstimator::is_enabled() const { return enabled_; }

void InekfEstimator::EnableFilter() { enabled_ = true; }

const bool InekfEstimator::BiasInitialized() const {
  if (propagation_.get()->get_propagation_type() != PropagationType::IMU) {
    return true;
  }

  std::shared_ptr<ImuPropagation> imu_propagation_ptr
      = std::dynamic_pointer_cast<ImuPropagation>(propagation_);
  return imu_propagation_ptr.get()->get_bias_initialized();
}

void InekfEstimator::InitBias() {
  if (propagation_.get()->get_propagation_type() != PropagationType::IMU) {
    return;
  }
  std::shared_ptr<ImuPropagation> imu_propagation_ptr
      = std::dynamic_pointer_cast<ImuPropagation>(propagation_);
  imu_propagation_ptr.get()->InitImuBias();
}

void InekfEstimator::InitState() {
  /// TODO: Implement clear filter
  // Clear filter
  this->clear();

  // Initialize state mean
  propagation_.get()->set_initial_state(state_);

  for (auto& correction_ : corrections_) {
    correction_.get()->set_initial_velocity(state_);
  }

  // Initialize state covariance
  state_.set_rotation_covariance(0.03 * Eigen::Matrix3d::Identity());
  state_.set_velocity_covariance(0.01 * Eigen::Matrix3d::Identity());
  state_.set_position_covariance(0.00001 * Eigen::Matrix3d::Identity());
  state_.set_gyroscope_bias_covariance(0.0001 * Eigen::Matrix3d::Identity());
  state_.set_accelerometer_bias_covariance(0.0025
                                           * Eigen::Matrix3d::Identity());

  std::cout << "Robot's state mean is initialized to: \n";
  std::cout << this->get_state().get_X() << std::endl;
  std::cout << "Robot's state covariance is initialized to: \n";
  std::cout << this->get_state().get_P() << std::endl;
  // Set enabled flag
  enabled_ = true;
}

void InekfEstimator::clear() {}
}    // namespace estimator
