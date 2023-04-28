/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   filtered_imu_propagation.cpp
 *  @author Tzu-Yuan Lin, Tingjun Li
 *  @brief  Source file for Invariant EKF imu propagation method
 *  Part of the code is modified from Ross Hartley's work:
 *  Paper:
 *  https://journals.sagepub.com/doi/full/10.1177/0278364919894385
 *  Github repo:
 *  https://github.com/RossHartley/invariant-ekf
 *
 *  @date   November 25, 2022
 **/

#include "imu_filter/imu_ang_vel_ekf.h"


using namespace std;
using namespace math::lie_group;

namespace imu_filter {

// Filtered IMU propagation child class
// ==============================================================================
// Filtered IMU propagation constructor
ImuAngVelEKF::ImuAngVelEKF(
    IMUQueuePtr imu_data_buffer_ptr,
    std::shared_ptr<std::mutex> imu_data_buffer_mutex_ptr,
    AngularVelocityQueuePtr angular_velocity_data_buffer_ptr,
    std::shared_ptr<std::mutex> angular_velocity_data_buffer_mutex_ptr,
    const std::string& yaml_filepath)
    : imu_data_buffer_ptr_(imu_data_buffer_ptr),
      imu_data_buffer_mutex_ptr_(imu_data_buffer_mutex_ptr),
      ang_vel_data_buffer_ptr_(angular_velocity_data_buffer_ptr),
      ang_vel_data_buffer_mutex_ptr_(angular_velocity_data_buffer_mutex_ptr),
      filtered_imu_data_buffer_ptr_(new IMUQueue),
      filtered_imu_data_buffer_mutex_ptr_(new std::mutex) {
  // Load configs
  cout << "Loading imu propagation config from " << yaml_filepath << endl;
  YAML::Node config_ = YAML::LoadFile(yaml_filepath);

  // Set the imu to body rotation
  const std::vector<double> quat_imu2body
      = config_["settings"]["rotation_imu2body"]
            ? config_["settings"]["rotation_imu2body"].as<std::vector<double>>()
            : std::vector<double>({1, 0, 0, 0});

  Eigen::Quaternion<double> quarternion_imu2body(
      quat_imu2body[0], quat_imu2body[1], quat_imu2body[2], quat_imu2body[3]);
  R_imu2body_ = quarternion_imu2body.toRotationMatrix();

  // Set the noise parameters
  double gyro_std = config_["noises"]["gyroscope_std"]
                        ? config_["noises"]["gyroscope_std"].as<double>()
                        : 0.1;
  double accel_std = config_["noises"]["accelerometer_std"]
                         ? config_["noises"]["accelerometer_std"].as<double>()
                         : 0.1;
  double gyro_bias_std
      = config_["noises"]["gyroscope_bias_std"]
            ? config_["noises"]["gyroscope_bias_std"].as<double>()
            : 0.1;
  double accel_bias_std
      = config_["noises"]["accelerometer_bias_std"]
            ? config_["noises"]["accelerometer_bias_std"].as<double>()
            : 0.1;
  double encoder_ang_vel_std
      = config_["noises"]["angular_velocity_std"]
            ? config_["noises"]["encoder_angular_velocity_std"].as<double>()
            : 0.1;
  double encoder_ang_vel_bias_std
      = config_["noises"]["angular_velocity_bias_std"]
            ? config_["noises"]["encoder_angular_velocity_bias_std"]
                  .as<double>()
            : 0.1;


  gyro_cov_ = gyro_std * gyro_std * Eigen::Matrix<double, 3, 3>::Identity();
  gyro_bias_cov_
      = gyro_bias_std * gyro_bias_std * Eigen::Matrix<double, 3, 3>::Identity();
  accel_cov_ = accel_std * accel_std * Eigen::Matrix<double, 3, 3>::Identity();
  accel_bias_cov_ = accel_bias_std * accel_bias_std
                    * Eigen::Matrix<double, 3, 3>::Identity();

  // Set the initial bias
  std::vector<double> gyroscope_bias
      = config_["priors"]["gyroscope_bias"]
            ? config_["priors"]["gyroscope_bias"].as<std::vector<double>>()
            : std::vector<double>({0, 0, 0});
  bg0_ = Eigen::Vector3d(gyroscope_bias[0], gyroscope_bias[1],
                         gyroscope_bias[2]);

  std::vector<double> accelerometer_bias
      = config_["priors"]["accelerometer_bias"]
            ? config_["priors"]["accelerometer_bias"].as<std::vector<double>>()
            : std::vector<double>({0, 0, 0});
  ba0_ = Eigen::Vector3d(accelerometer_bias[0], accelerometer_bias[1],
                         accelerometer_bias[2]);

  // Set the parameters for the angular velocity filter
  H_imu_.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3);
  H_imu_.block<3, 3>(0, 3) = Eigen::MatrixXd::Identity(3, 3);
  H_enc_.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3);
  H_enc_.block<3, 3>(0, 3) = Eigen::MatrixXd::Zero(3, 3);

  std::string imu_ang_vel_log_file
      = "/home/tingjunl/code/drift/log/imu_ang_vel_log.txt";
  imu_ang_vel_outfile_.open(imu_ang_vel_log_file);
  imu_ang_vel_outfile_.precision(dbl::max_digits10);

  std::string encoder_ang_vel_log_file
      = "/home/tingjunl/code/drift/log/encoder_ang_vel_log.txt";
  encoder_ang_vel_outfile_.open(encoder_ang_vel_log_file);
  encoder_ang_vel_outfile_.precision(dbl::max_digits10);

  std::string filtered_ang_vel_log_file
      = "/home/tingjunl/code/drift/log/filtered_ang_vel_log.txt";
  filtered_ang_vel_outfile_.open(filtered_ang_vel_log_file);
  filtered_ang_vel_outfile_.precision(dbl::max_digits10);
}

// IMU filter destructor
ImuAngVelEKF::~ImuAngVelEKF() {
  stop_thread_ = true;
  imu_filter_thread_.join();
  imu_ang_vel_outfile_.close();
  encoder_ang_vel_outfile_.close();
  filtered_ang_vel_outfile_.close();
}

// Start IMU filter thread
void ImuAngVelEKF::StartImuFilterThread() {
  std::cout << "Starting IMU filter thread..." << std::endl;
  this->imu_filter_thread_ = std::thread([this] { this->RunFilter(); });
}

void ImuAngVelEKF::RunFilter() {
  while (!stop_thread_) {
    RunOnce();
  }
}

// IMU propagation method
void ImuAngVelEKF::RunOnce() {
  ImuMeasurementPtr imu_measurement = nullptr;
  AngularVelocityMeasurementPtr ang_vel_measurement = nullptr;
  // Bias corrected IMU measurements
  imu_data_buffer_mutex_ptr_.get()->lock();
  if (imu_data_buffer_ptr_->empty()) {
    imu_data_buffer_mutex_ptr_.get()->unlock();
  } else {
    imu_measurement = imu_data_buffer_ptr_->front();
    imu_data_buffer_mutex_ptr_.get()->unlock();
  }
  // Angular velocity measurements
  ang_vel_data_buffer_mutex_ptr_.get()->lock();
  if (ang_vel_data_buffer_ptr_->empty()) {
    ang_vel_data_buffer_mutex_ptr_.get()->unlock();
  } else {
    ang_vel_measurement = ang_vel_data_buffer_ptr_->front();
    ang_vel_data_buffer_mutex_ptr_.get()->unlock();
  }

  if (!imu_measurement && !ang_vel_measurement) {
    return;
  }

  if (imu_measurement && ang_vel_measurement) {
    // Only use the earliest sensor data and keep the other one in the buffer
    if (imu_measurement->get_time() <= ang_vel_measurement->get_time()) {
      ang_vel_measurement = nullptr;
    } else {
      imu_measurement = nullptr;
    }
  }
  // =================== Angular Velocity filter ====================
  AngVelFilterPropagate();
  if (imu_measurement) {
    latest_imu_measurement_ = imu_measurement;
    auto fused_ang_imu = AngVelFilterCorrectIMU(imu_measurement);

    // std::cout << "IMU filter running..." << std::endl;
    filtered_imu_data_buffer_mutex_ptr_.get()->lock();
    filtered_imu_data_buffer_ptr_->push(fused_ang_imu);
    filtered_imu_data_buffer_mutex_ptr_.get()->unlock();

    imu_data_buffer_mutex_ptr_.get()->lock();
    imu_data_buffer_ptr_->pop();
    imu_data_buffer_mutex_ptr_.get()->unlock();
    return;
  }

  if (ang_vel_measurement && !latest_imu_measurement_) {
    ang_vel_data_buffer_mutex_ptr_.get()->lock();
    ang_vel_data_buffer_ptr_->pop();
    ang_vel_data_buffer_mutex_ptr_.get()->unlock();
    return;
  }

  // only execute the following code if we have a latest_imu_measurement_
  if (ang_vel_measurement && latest_imu_measurement_) {
    auto fused_ang_imu = AngVelFilterCorrectEncoder(latest_imu_measurement_,
                                                    ang_vel_measurement);
    // std::cout << "Encoder filter running..." << std::endl;
    filtered_imu_data_buffer_mutex_ptr_.get()->lock();
    filtered_imu_data_buffer_ptr_->push(fused_ang_imu);
    filtered_imu_data_buffer_mutex_ptr_.get()->unlock();

    ang_vel_data_buffer_mutex_ptr_.get()->lock();
    ang_vel_data_buffer_ptr_->pop();
    ang_vel_data_buffer_mutex_ptr_.get()->unlock();
    return;
  }
}


void ImuAngVelEKF::AngVelFilterPropagate() {
  // Random walk
  // X = X, so we don't need to do anything for mean propagation

  // Covariance propagation
  ang_vel_and_bias_P_ = ang_vel_and_bias_P_ + ang_vel_and_bias_Q_;
}

ImuMeasurementPtr ImuAngVelEKF::AngVelFilterCorrectIMU(
    const ImuMeasurementPtr& imu_measurement) {
  Eigen::VectorXd z
      = imu_measurement->get_ang_vel() - H_imu_ * ang_vel_and_bias_est_;
  auto S_inv = (H_imu_ * ang_vel_and_bias_P_ * H_imu_.transpose()
                + ang_vel_and_bias_imu_R_)
                   .inverse();
  auto K = ang_vel_and_bias_P_ * H_imu_.transpose() * S_inv;
  ang_vel_and_bias_est_ = ang_vel_and_bias_est_ + K * z;
  ang_vel_and_bias_P_
      = (Eigen::MatrixXd::Identity(6, 6) - K * H_imu_) * ang_vel_and_bias_P_;

  ImuMeasurementPtr imu_measurement_corrected
      = std::make_shared<ImuMeasurement<double>>(*imu_measurement);
  imu_measurement_corrected->set_ang_vel(ang_vel_and_bias_est_(0),
                                         ang_vel_and_bias_est_(1),
                                         ang_vel_and_bias_est_(2));
  imu_measurement_corrected->set_time(imu_measurement->get_time());

  imu_ang_vel_outfile_ << imu_measurement->get_time() << ","
                       << imu_measurement->get_ang_vel()(0) << ","
                       << imu_measurement->get_ang_vel()(1) << ","
                       << imu_measurement->get_ang_vel()(2) << std::endl
                       << std::flush;

  filtered_ang_vel_outfile_
      << imu_measurement->get_time() << "," << ang_vel_and_bias_est_(0) << ","
      << ang_vel_and_bias_est_(1) << "," << ang_vel_and_bias_est_(2) << ","
      << ang_vel_and_bias_est_(3) << "," << ang_vel_and_bias_est_(4) << ","
      << ang_vel_and_bias_est_(5) << std::endl
      << std::flush;
  return imu_measurement_corrected;
}

ImuMeasurementPtr ImuAngVelEKF::AngVelFilterCorrectEncoder(
    const ImuMeasurementPtr& imu_measurement,
    const AngularVelocityMeasurementPtr& ang_vel_measurement) {
  Eigen::VectorXd z
      = ang_vel_measurement->get_ang_vel() - H_enc_ * ang_vel_and_bias_est_;
  auto S_inv = (H_enc_ * ang_vel_and_bias_P_ * H_enc_.transpose()
                + ang_vel_and_bias_enc_R_)
                   .inverse();
  auto K = ang_vel_and_bias_P_ * H_enc_.transpose() * S_inv;
  ang_vel_and_bias_est_ = ang_vel_and_bias_est_ + K * z;
  ang_vel_and_bias_P_
      = (Eigen::MatrixXd::Identity(6, 6) - K * H_enc_) * ang_vel_and_bias_P_;

  ImuMeasurementPtr imu_measurement_corrected
      = std::make_shared<ImuMeasurement<double>>(*imu_measurement);
  imu_measurement_corrected->set_ang_vel(ang_vel_and_bias_est_(0),
                                         ang_vel_and_bias_est_(1),
                                         ang_vel_and_bias_est_(2));
  imu_measurement_corrected->set_time(ang_vel_measurement->get_time());

  encoder_ang_vel_outfile_ << ang_vel_measurement->get_time() << ","
                           << ang_vel_measurement->get_ang_vel()(0) << ","
                           << ang_vel_measurement->get_ang_vel()(1) << ","
                           << ang_vel_measurement->get_ang_vel()(2) << std::endl
                           << std::flush;

  filtered_ang_vel_outfile_
      << imu_measurement->get_time() << "," << ang_vel_and_bias_est_(0) << ","
      << ang_vel_and_bias_est_(1) << "," << ang_vel_and_bias_est_(2) << ","
      << ang_vel_and_bias_est_(3) << "," << ang_vel_and_bias_est_(4) << ","
      << ang_vel_and_bias_est_(5) << std::endl
      << std::flush;

  return imu_measurement_corrected;
}

// Getters
IMUQueuePtr ImuAngVelEKF::get_filtered_imu_data_buffer_ptr() {
  return filtered_imu_data_buffer_ptr_;
}

std::shared_ptr<std::mutex>
ImuAngVelEKF::get_filtered_imu_data_buffer_mutex_ptr() {
  return filtered_imu_data_buffer_mutex_ptr_;
}
}    // namespace imu_filter