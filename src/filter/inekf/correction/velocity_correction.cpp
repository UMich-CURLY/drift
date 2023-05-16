/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   velocity_correction.cpp
 *  @author Tingjun Li, Tzu-Yuan Lin
 *  @brief  Source file for Invariant EKF velocity correction method
 *  @date   November 25, 2022
 **/

#include "drift/filter/inekf/correction/velocity_correction.h"

using namespace std;
using namespace math::lie_group;

namespace filter::inekf {

VelocityCorrection::VelocityCorrection(
    VelocityQueuePtr sensor_data_buffer_ptr,
    std::shared_ptr<std::mutex> sensor_data_buffer_mutex_ptr,
    const ErrorType& error_type, const string& yaml_filepath)
    : Correction::Correction(sensor_data_buffer_mutex_ptr),
      sensor_data_buffer_ptr_(sensor_data_buffer_ptr),
      error_type_(error_type) {
  correction_type_ = CorrectionType::VELOCITY;

  cout << "Loading velocity correction config from " << yaml_filepath << endl;
  YAML::Node config_ = YAML::LoadFile(yaml_filepath);

  // Set noises:
  double std = config_["noises"]["velocity_std"]
                   ? config_["noises"]["velocity_std"].as<double>()
                   : 0.1;
  covariance_ = std * std * Eigen::Matrix<double, 3, 3>::Identity();

  // Set thresholds:
  t_diff_thres_
      = config_["settings"]["correction_time_threshold"]
            ? config_["settings"]["correction_time_threshold"].as<double>()
            : 0.3;

  // Set rotation matrix from velocity to body frame:
  const std::vector<double> quat_vel2body
      = config_["settings"]["rotation_vel2body"]
            ? config_["settings"]["rotation_vel2body"].as<std::vector<double>>()
            : std::vector<double>({1, 0, 0, 0});

  // Convert quaternion to rotation matrix for frame transformation
  Eigen::Quaternion<double> quarternion_vel2body(
      quat_vel2body[0], quat_vel2body[1], quat_vel2body[2], quat_vel2body[3]);
  R_vel2body_ = quarternion_vel2body.toRotationMatrix();
}


const VelocityQueuePtr VelocityCorrection::get_sensor_data_buffer_ptr() const {
  return sensor_data_buffer_ptr_;
}

// Correct using measured body velocity with the estimated velocity
bool VelocityCorrection::Correct(RobotState& state) {
  Eigen::VectorXd Z, Y, b;
  Eigen::MatrixXd H, N, PI;

  // Fill out observation data
  int dimX = state.dimX();
  int dimTheta = state.dimTheta();
  int dimP = state.dimP();

  // Get latest measurement:
  sensor_data_buffer_mutex_ptr_->lock();
  if (sensor_data_buffer_ptr_->empty()) {
    sensor_data_buffer_mutex_ptr_->unlock();
    return false;
  }
  VelocityMeasurementPtr measured_velocity = sensor_data_buffer_ptr_->front();
  double t_diff = measured_velocity->get_time() - state.get_propagate_time();
  // Only use previous velocity measurement to correct the state
  if (t_diff >= 0) {
    sensor_data_buffer_mutex_ptr_->unlock();
    return false;
  }

  sensor_data_buffer_ptr_->pop();
  sensor_data_buffer_mutex_ptr_->unlock();

  if (t_diff < -t_diff_thres_) {
    while (t_diff < -t_diff_thres_) {
      sensor_data_buffer_mutex_ptr_->lock();
      if (sensor_data_buffer_ptr_->empty()) {
        sensor_data_buffer_mutex_ptr_->unlock();
        return false;
      }
      measured_velocity = sensor_data_buffer_ptr_->front();
      sensor_data_buffer_ptr_->pop();
      sensor_data_buffer_mutex_ptr_->unlock();

      t_diff = measured_velocity->get_time() - state.get_propagate_time();
    }
  }

  state.set_time(measured_velocity->get_time());

  // Fill out H
  H.conservativeResize(3, dimP);
  H.block(0, 0, 3, dimP) = Eigen::MatrixXd::Zero(3, dimP);
  H.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity();

  // Fill out N
  N.conservativeResize(3, 3);
  N = covariance_;

  Eigen::Matrix3d R = state.get_rotation();
  Eigen::Vector3d v = state.get_velocity();

  int startIndex = Z.rows();
  Z.conservativeResize(startIndex + 3, Eigen::NoChange);
  // Rotate the velocity from sensor frame to body frame, then to world frame
  Z.segment(0, 3) = R * R_vel2body_ * measured_velocity->get_velocity() - v;

  // Correct state using stacked observation
  if (Z.rows() > 0) {
    CorrectRightInvariant(Z, H, N, state, error_type_);
  }

  return true;
}

bool VelocityCorrection::set_initial_velocity(RobotState& state) {
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();

  // Get measurement from sensor data buffer
  // Do not initialize if the buffer is emptys
  while (sensor_data_buffer_ptr_->empty()) {
    // std::cout << "Waiting for velocity related encoder data..." << std::endl;
    return false;
  }

  sensor_data_buffer_mutex_ptr_.get()->lock();
  // Get the latest measurement
  while (sensor_data_buffer_ptr_->size() > 1) {
    // std::cout << "Discarding old sensor data..." << std::endl;
    sensor_data_buffer_ptr_->pop();
  }
  VelocityMeasurementPtr measured_velocity = sensor_data_buffer_ptr_->front();
  sensor_data_buffer_ptr_->pop();
  sensor_data_buffer_mutex_ptr_.get()->unlock();

  // Apply the rotation to map the body velocity to the world frame
  state.set_velocity(state.get_rotation() * measured_velocity->get_velocity());
  state.set_time(measured_velocity->get_time());
  return true;
}

void VelocityCorrection::clear() {
  sensor_data_buffer_mutex_ptr_->lock();
  while (!sensor_data_buffer_ptr_->empty()) {
    sensor_data_buffer_ptr_->pop();
  }
  sensor_data_buffer_mutex_ptr_->unlock();
}
}    // namespace filter::inekf