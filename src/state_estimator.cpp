/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   state_estimator.cpp
 *  @author Tzu-Yuan Lin, Tingjun Li
 *  @brief  Source file for state estimator class
 *  @date   December 1, 2022
 **/

#include "state_estimator.h"

StateEstimator::StateEstimator()
    : robot_state_queue_ptr_(new RobotStateQueue),
      robot_state_queue_mutex_ptr_(new std::mutex) {}

StateEstimator::StateEstimator(ErrorType error_type)
    : error_type_(error_type),
      robot_state_queue_ptr_(new RobotStateQueue),
      robot_state_queue_mutex_ptr_(new std::mutex) {}

void StateEstimator::RunOnce() {
  // Propagate
  new_pose_ready_ = propagation_.get()->Propagate(state_);
  // std::cout << "Propagated state: \n"
  //           << state_.get_position().transpose() << std::endl;

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

RobotStateQueuePtr StateEstimator::get_robot_state_queue_ptr() {
  return robot_state_queue_ptr_;
}

std::shared_ptr<std::mutex> StateEstimator::get_robot_state_queue_mutex_ptr() {
  return robot_state_queue_mutex_ptr_;
}

void StateEstimator::set_state(RobotState& state) { state_ = state; }

const RobotState StateEstimator::get_state() const { return state_; }

void StateEstimator::add_imu_propagation(
    IMUQueuePtr buffer_ptr, std::shared_ptr<std::mutex> buffer_mutex_ptr,
    const std::string& yaml_filepath) {
  propagation_ = std::make_shared<ImuPropagation>(buffer_ptr, buffer_mutex_ptr,
                                                  error_type_, yaml_filepath);
}

void StateEstimator::add_legged_kinematics_correction(
    LeggedKinematicsQueuePtr buffer_ptr,
    std::shared_ptr<std::mutex> buffer_mutex_ptr,
    const std::string& yaml_filepath) {
  std::shared_ptr<Correction> correction
      = std::make_shared<LeggedKinematicsCorrection>(
          buffer_ptr, buffer_mutex_ptr, error_type_, yaml_filepath);
  corrections_.push_back(correction);
}

void StateEstimator::add_velocity_correction(
    VelocityQueuePtr buffer_ptr, std::shared_ptr<std::mutex> buffer_mutex_ptr,
    const std::string& yaml_filepath) {
  std::shared_ptr<Correction> correction = std::make_shared<VelocityCorrection>(
      buffer_ptr, buffer_mutex_ptr, error_type_, yaml_filepath);
  corrections_.push_back(correction);
}

const bool StateEstimator::is_enabled() const { return enabled_; }

void StateEstimator::EnableFilter() { enabled_ = true; }

const bool StateEstimator::BiasInitialized() const {
  if (propagation_.get()->get_propagation_type() != PropagationType::IMU) {
    return true;
  }

  std::shared_ptr<ImuPropagation> imu_propagation_ptr
      = std::dynamic_pointer_cast<ImuPropagation>(propagation_);
  return imu_propagation_ptr.get()->get_bias_initialized();
}

void StateEstimator::InitBias() {
  if (propagation_.get()->get_propagation_type() != PropagationType::IMU) {
    return;
  }
  std::shared_ptr<ImuPropagation> imu_propagation_ptr
      = std::dynamic_pointer_cast<ImuPropagation>(propagation_);
  imu_propagation_ptr.get()->InitImuBias();
}

void StateEstimator::InitStateFromImu() {
  /// TODO: Implement clear filter
  // Clear filter
  this->clear();

  // Initialize state mean
  std::shared_ptr<ImuPropagation> imu_propagation_ptr
      = std::dynamic_pointer_cast<ImuPropagation>(propagation_);
  const IMUQueuePtr imu_queue_ptr
      = imu_propagation_ptr.get()->get_sensor_data_buffer_ptr();

  imu_propagation_ptr.get()->get_mutex_ptr()->lock();

  // Don't initialize if IMU queue is empty
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

  Eigen::Vector3d v0_body = Eigen::Vector3d::Zero();
  for (auto& correction : corrections_) {
    /// TODO: How to deal with multiple velocity measurements?
    if (correction.get()->get_correction_type() == CorrectionType::VELOCITY) {
      std::shared_ptr<VelocityCorrection> velocity_correction_ptr
          = std::dynamic_pointer_cast<VelocityCorrection>(correction);
      const VelocityQueuePtr velocity_queue_ptr
          = velocity_correction_ptr.get()->get_sensor_data_buffer_ptr();

      velocity_correction_ptr.get()->get_mutex_ptr()->lock();
      if (velocity_queue_ptr.get()->empty()) {
        // std::cout << "Velocity queue is empty, cannot initialize state"
        velocity_correction_ptr.get()->get_mutex_ptr()->unlock();
        // << std::endl;
        return;
      }
      const VelocityMeasurementPtr velocity_packet_in
          = velocity_queue_ptr->front();
      velocity_queue_ptr->pop();
      velocity_correction_ptr.get()->get_mutex_ptr()->unlock();

      v0_body = velocity_packet_in->get_velocity();
      break;
    }
  }

  Eigen::Vector3d v0 = R0 * v0_body;    // initial velocity

  Eigen::Vector3d p0
      = {0.0, 0.0, 0.0};    // initial position, we set imu frame as world frame

  R0 = Eigen::Matrix3d::Identity();
  RobotState initial_state;
  Eigen::Vector3d bg0 = imu_propagation_ptr.get()->get_estimate_gyro_bias();
  Eigen::Vector3d ba0 = imu_propagation_ptr.get()->get_estimate_accel_bias();
  initial_state.set_rotation(R0);
  initial_state.set_velocity(v0);
  initial_state.set_position(p0);
  initial_state.set_gyroscope_bias(bg0);
  initial_state.set_accelerometer_bias(ba0);
  // Initialize state covariance
  initial_state.set_rotation_covariance(0.03 * Eigen::Matrix3d::Identity());
  initial_state.set_velocity_covariance(0.01 * Eigen::Matrix3d::Identity());
  initial_state.set_position_covariance(0.00001 * Eigen::Matrix3d::Identity());
  initial_state.set_gyroscope_bias_covariance(0.0001
                                              * Eigen::Matrix3d::Identity());
  initial_state.set_accelerometer_bias_covariance(
      0.0025 * Eigen::Matrix3d::Identity());
  this->set_state(initial_state);
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

void StateEstimator::clear() {}
