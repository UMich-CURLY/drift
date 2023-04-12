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
  std::shared_ptr<ImuPropagation> imu_propagation_ptr
      = std::dynamic_pointer_cast<ImuPropagation>(propagation_);
  const IMUQueuePtr imu_queue_ptr
      = imu_propagation_ptr.get()->get_sensor_data_buffer_ptr();

  // Don't initialize if IMU queue is empty
  imu_propagation_ptr.get()->get_mutex_ptr()->lock();
  if (imu_queue_ptr.get()->empty()) {
    imu_propagation_ptr.get()->get_mutex_ptr()->unlock();
    return;
  }

  const ImuMeasurementPtr imu_packet_in = imu_queue_ptr->front();
  imu_queue_ptr->pop();
  imu_propagation_ptr.get()->get_mutex_ptr()->unlock();

  // Eigen::Quaternion<double> quat = imu_packet_in->get_quaternion();
  // Eigen::Matrix3d R0 = quat.toRotationMatrix(); // Initialize based on
  // VectorNav estimate
  Eigen::Matrix3d R0 = Eigen::Matrix3d::Identity();
  Eigen::Vector3d w0 = imu_packet_in->get_ang_vel();

  Eigen::Vector3d v0_body = Eigen::Vector3d::Zero();
  for (auto& correction : corrections_) {
    /// TODO: How to deal with multiple velocity measurements?
    if (correction.get()->get_correction_type() == CorrectionType::VELOCITY) {
      std::shared_ptr<VelocityCorrection> velocity_correction_ptr
          = std::dynamic_pointer_cast<VelocityCorrection>(correction);
      v0_body
          = velocity_correction_ptr.get()->set_initial_velocity(w0, state_, R0);
      break;
    } else if (correction.get()->get_correction_type()
               == CorrectionType::LEGGED_KINEMATICS) {
      std::shared_ptr<LeggedKinematicsCorrection> legged_correction_ptr
          = std::dynamic_pointer_cast<LeggedKinematicsCorrection>(correction);
      v0_body = legged_correction_ptr.get()->get_initial_velocity(w0);
      break;
    }
  }

  Eigen::Vector3d v0 = R0 * v0_body;    // initial velocity

  Eigen::Vector3d p0
      = {0.0, 0.0, 0.0};    // initial position, we set imu frame as world frame

  Eigen::Vector3d bg0 = imu_propagation_ptr.get()->get_estimate_gyro_bias();
  Eigen::Vector3d ba0 = imu_propagation_ptr.get()->get_estimate_accel_bias();
  state_.set_rotation(R0);
  state_.set_velocity(v0);
  state_.set_position(p0);
  state_.set_gyroscope_bias(bg0);
  state_.set_accelerometer_bias(ba0);
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
  double t_prev = imu_packet_in->get_time();
  state_.set_time(t_prev);
  state_.set_propagate_time(t_prev);
  enabled_ = true;
}

void InekfEstimator::clear() {}
}    // namespace estimator
