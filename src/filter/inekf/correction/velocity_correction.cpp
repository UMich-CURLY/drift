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

  velocity_scale_ = config_["settings"]["velocity_scale"]
                        ? config_["settings"]["velocity_scale"].as<double>()
                        : 1.0;

  // Convert quaternion to rotation matrix for frame transformation
  Eigen::Quaternion<double> quarternion_vel2body(
      quat_vel2body[0], quat_vel2body[1], quat_vel2body[2], quat_vel2body[3]);
  R_vel2body_ = quarternion_vel2body.toRotationMatrix();


  std::string est_vel_file
      = "/home/justin/code/drift/log/vanila_est_vel_log.txt";
  est_vel_outfile_.open(est_vel_file);
  est_vel_outfile_.precision(dbl::max_digits10);
}

VelocityCorrection::~VelocityCorrection() { est_vel_outfile_.close(); }

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
      std::cout << "entered while loop" << std::endl;
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

  // Open the file in append mode and write the data
  std::ofstream file("debugging_data.txt", std::ios::app);
  if (file.is_open()) {
    // Ensure fixed decimal format with three decimal places for positions
    file << std::fixed << std::setprecision(6);

    // Get time in seconds with milliseconds precision
    double filter_time = state.get_propagate_time();
    double measurement_time = measured_velocity->get_time();

    // Write Filter time
    file << "Filter time: " << std::fixed << std::setprecision(6) << filter_time
         << " ";

    // Write Filter state by extracting components explicitly
    Eigen::Vector3d filter_state = state.get_velocity();
    file << "Filter state: " << std::fixed << std::setprecision(6)
         << filter_state.x() << " " << filter_state.y() << " "
         << filter_state.z() << "; ";

    // Write Measurement time
    file << "Measurement time: " << std::fixed << std::setprecision(6)
         << measurement_time << " ";

    // Write Measurement state by extracting components explicitly
    Eigen::Vector3d measurement_state = measured_velocity->get_velocity();
    file << "Measurement state: " << std::fixed << std::setprecision(6)
         << measurement_state.x() << " " << measurement_state.y() << " "
         << measurement_state.z() << "\n";

    file.close();
  } else {
    std::cerr << "Unable to open file for writing imu data.\n";
  }
  state.set_time(measured_velocity->get_time());

  // Fill out H
  H.conservativeResize(3, dimP);
  H.block(0, 0, 3, dimP) = Eigen::MatrixXd::Zero(3, dimP);
  H.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity();
  // std::cout << "H matrix: \n" << H;
  // Fill out N
  N.conservativeResize(3, 3);
  N = state.get_world_rotation() * covariance_
      * state.get_world_rotation().transpose();

  // std::cout << "Measurement noise covariance: \n" << N;
  Eigen::Matrix3d R = state.get_rotation();
  Eigen::Vector3d v = state.get_velocity();
  // std::cout << "Predicted velocity: \n" << state.get_velocity();
  // std::cout << "Measured velocity (before): "
  //           << measured_velocity->get_velocity() << "\n";
  // std::cout << "Measured velocity (after): "
  //           << R * R_vel2body_ * velocity_scale_
  //                  * measured_velocity->get_velocity()
  //           << "\n";
  int startIndex = Z.rows();
  Z.conservativeResize(startIndex + 3, Eigen::NoChange);
  // Rotate the velocity from sensor frame to body frame, then to world frame
  Z.segment(0, 3)
      = R * R_vel2body_ * velocity_scale_ * measured_velocity->get_velocity()
        - v;
  // std::cout << "Z matrix: \n" << Z;
  // Correct state using stacked observation
  if (Z.rows() > 0) {
    CorrectRightInvariant(Z, H, N, state, error_type_);
  }

  v = (R * R_vel2body_).inverse() * state.get_velocity();
  // std::cout << "Estimated velocity: \n" << state.get_velocity();
  est_vel_outfile_ << measured_velocity->get_time() << "," << v(0) << ","
                   << v(1) << "," << v(2) << std::endl
                   << std::flush;

  return true;
}

bool VelocityCorrection::initialize(RobotState& state) {
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