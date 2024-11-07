/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   inekf_estimator.cpp
 *  @author Tzu-Yuan Lin, Tingjun Li
 *  @brief  Source file for InEKF estimator class
 *  @date   May 15, 2023
 **/

#include "drift/estimator/inekf_estimator.h"

namespace estimator {
InekfEstimator::InekfEstimator()
    : robot_state_queue_ptr_(new RobotStateQueue),
      robot_state_queue_mutex_ptr_(new std::mutex),
      robot_state_log_queue_ptr_(new RobotStateQueue),
      robot_state_log_queue_mutex_() {}

InekfEstimator::InekfEstimator(ErrorType error_type, std::string config_file)
    : error_type_(error_type),
      robot_state_queue_ptr_(new RobotStateQueue),
      robot_state_queue_mutex_ptr_(new std::mutex),
      robot_state_log_queue_ptr_(new RobotStateQueue),
      robot_state_log_queue_mutex_() {
  YAML::Node config = YAML::LoadFile(config_file);
  enable_pose_logger_ = config["logger"]["enable_pose_logger"].as<bool>();

  rotation_cov_val_
      = config["state"]["rotation_std_value"]
            ? pow(config["state"]["rotation_std_value"].as<double>(), 2)
            : 0.03;
  velocity_cov_val_
      = config["state"]["velocity_std_value"]
            ? pow(config["state"]["velocity_std_value"].as<double>(), 2)
            : 0.01;
  position_cov_val_
      = config["state"]["position_std_value"]
            ? pow(config["state"]["position_std_value"].as<double>(), 2)
            : 0.00001;

  if (enable_pose_logger_) {
    pose_log_file_ = config["logger"]["pose_log_file"]
                         ? config["logger"]["pose_log_file"].as<std::string>()
                         : "inekf_pose";
    vel_log_file_ = config["logger"]["vel_log_file"]
                        ? config["logger"]["vel_log_file"].as<std::string>()
                        : "inekf_vel";
    // Erase the ".txt" or ".csv" from the file name.
    // This is helpful to make new file names when InekfEstimator::clear() is
    // called
    std::string::size_type pos = pose_log_file_.find_last_of(".");
    if (pos != std::string::npos)
      pose_log_file_.erase(pos, pose_log_file_.length());


    std::string::size_type v_pos = vel_log_file_.find_last_of(".");
    if (v_pos != std::string::npos)
      vel_log_file_.erase(v_pos, vel_log_file_.length());

    // Define logger settings
    pose_log_rate_ = config["logger"]["pose_log_rate"]
                         ? config["logger"]["pose_log_rate"].as<double>()
                         : 10.0;
    outfile_.open(pose_log_file_ + ".txt");
    outfile_.precision(dbl::max_digits10);

    vel_outfile_.open(vel_log_file_ + ".txt");
    vel_outfile_.precision(dbl::max_digits10);

    vel_body_outfile_.open(vel_log_file_ + "_body_frame.txt");
    vel_body_outfile_.precision(dbl::max_digits10);

    this->StartLoggingThread();
  }
}

InekfEstimator::~InekfEstimator() {
  if (enable_pose_logger_) {
    std::cout << "Saving logged path..." << std::endl;
    stop_signal_ = true;
    this->pose_logging_thread_.join();
    std::cout << "Logged pose is saved to " << pose_log_file_ << "*.txt"
              << std::endl;
    std::cout << "Logged velocity (world frame) is saved to " << vel_log_file_
              << ".txt" << std::endl;
    std::cout << "Logged velocity (body frame) is saved to " << vel_log_file_
              << "_body_frame.txt" << std::endl;
    outfile_.close();
    vel_outfile_.close();
    vel_body_outfile_.close();
  }
}

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

    if (enabled_slip_estimator_)
      this->SlipEstimatorStep();

    if (enable_pose_logger_) {
      robot_state_log_queue_mutex_.lock();
      robot_state_log_queue_ptr_.get()->push(
          std::make_shared<RobotState>(state_));
      robot_state_log_queue_mutex_.unlock();
    }
  }
  new_pose_ready_ = false;
}

void InekfEstimator::StartLoggingThread() {
  if (enable_pose_logger_) {
    std::cout << "Starting logging thread..." << std::endl;
    this->pose_logging_thread_
        = std::thread([this] { this->PoseLoggingThread(); });
  }
}

void InekfEstimator::PoseLoggingThread() {
  // logger, tum style
  while (stop_signal_ == false) {
    // Get new pose data for logger
    robot_state_log_queue_mutex_.lock();
    if (robot_state_log_queue_ptr_.get()->empty()) {
      robot_state_log_queue_mutex_.unlock();
      continue;
    }

    std::shared_ptr<RobotState> state_log_ptr
        = robot_state_log_queue_ptr_.get()->front();
    robot_state_log_queue_ptr_.get()->pop();
    robot_state_log_queue_mutex_.unlock();

    double time_diff = state_log_ptr->get_time() - last_pub_t_;
    if (enable_pose_logger_ && time_diff >= 1.0 / pose_log_rate_) {
      Eigen::Quaterniond state_quat(state_log_ptr->get_world_rotation());
      outfile_ << state_log_ptr->get_time() << " "
               << state_log_ptr->get_world_position()(0) << " "
               << state_log_ptr->get_world_position()(1) << " "
               << state_log_ptr->get_world_position()(2) << " "
               << state_quat.x() << " " << state_quat.y() << " "
               << state_quat.z() << " " << state_quat.w() << std::endl
               << std::flush;

      vel_outfile_ << state_log_ptr->get_time() << " "
                   << state_log_ptr->get_world_velocity()(0) << " "
                   << state_log_ptr->get_world_velocity()(1) << " "
                   << state_log_ptr->get_world_velocity()(2) << " " << std::endl
                   << std::flush;


      vel_body_outfile_ << state_log_ptr->get_time() << " "
                        << state_log_ptr->get_body_velocity()(0) << " "
                        << state_log_ptr->get_body_velocity()(1) << " "
                        << state_log_ptr->get_body_velocity()(2) << " "
                        << std::endl
                        << std::flush;
      last_pub_t_ = state_log_ptr->get_time();
    }
  }
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
  propagation_ = std::make_shared<ImuPropagation>(buffer_ptr, buffer_mutex_ptr,
                                                  error_type_, yaml_filepath);
}

std::pair<IMUQueuePtr, std::shared_ptr<std::mutex>>
InekfEstimator::add_imu_ang_vel_ekf(
    IMUQueuePtr imu_buffer_ptr,
    std::shared_ptr<std::mutex> imu_buffer_mutex_ptr,
    AngularVelocityQueuePtr ang_vel_buffer_ptr,
    std::shared_ptr<std::mutex> ang_vel_buffer_mutex_ptr,
    const std::string& yaml_filepath) {
  imu_filter_ = std::make_shared<imu_filter::ImuAngVelEKF>(
      imu_buffer_ptr, imu_buffer_mutex_ptr, ang_vel_buffer_ptr,
      ang_vel_buffer_mutex_ptr, yaml_filepath);

  imu_filter_.get()->StartImuFilterThread();

  return {imu_filter_.get()->get_filtered_imu_data_buffer_ptr(),
          imu_filter_.get()->get_filtered_imu_data_buffer_mutex_ptr()};
}

void InekfEstimator::add_legged_kinematics_correction(
    LeggedKinQueuePtr buffer_ptr, std::shared_ptr<std::mutex> buffer_mutex_ptr,
    const std::string& yaml_filepath) {
  std::shared_ptr<Correction> correction
      = std::make_shared<LeggedKinematicsCorrection>(
          buffer_ptr, buffer_mutex_ptr, error_type_, yaml_filepath);
  corrections_.push_back(correction);
}

void InekfEstimator::add_velocity_correction(
    VelocityQueuePtr buffer_ptr, std::shared_ptr<std::mutex> buffer_mutex_ptr,
    const std::string& yaml_filepath) {
  std::shared_ptr<Correction> correction = std::make_shared<VelocityCorrection>(
      buffer_ptr, buffer_mutex_ptr, error_type_, yaml_filepath);
  corrections_.push_back(correction);
}

void InekfEstimator::add_position_correction(
    OdomQueuePtr buffer_ptr, std::shared_ptr<std::mutex> buffer_mutex_ptr,
    const std::string& yaml_filepath) {
  std::shared_ptr<Correction> correction = std::make_shared<PositionCorrection>(
      buffer_ptr, buffer_mutex_ptr, error_type_, yaml_filepath);
  corrections_.push_back(correction);
}

const bool InekfEstimator::is_enabled() const { return enabled_; }

void InekfEstimator::EnableFilter() { enabled_ = true; }

const bool InekfEstimator::BiasInitialized() const {
  if (propagation_.get()->get_propagation_type() == PropagationType::IMU) {
    std::shared_ptr<ImuPropagation> imu_propagation_ptr
        = std::dynamic_pointer_cast<ImuPropagation>(propagation_);
    return imu_propagation_ptr.get()->get_bias_initialized();
  }
  return false;
}

void InekfEstimator::InitBias() {
  if (propagation_.get()->get_propagation_type() == PropagationType::IMU) {
    std::shared_ptr<ImuPropagation> imu_propagation_ptr
        = std::dynamic_pointer_cast<ImuPropagation>(propagation_);
    imu_propagation_ptr.get()->InitImuBias();
  }
}

void InekfEstimator::InitState() {
  // Initialize state mean
  bool propagation_initialized = propagation_.get()->set_initial_state(state_);
  if (!propagation_initialized) {
    return;
  }

  for (auto& correction_ : corrections_) {
    bool current_correction_initialized = false;
    while (!current_correction_initialized)
      current_correction_initialized = correction_.get()->initialize(state_);
  }

  // Initialize state covariance
  state_.set_rotation_covariance(rotation_cov_val_
                                 * Eigen::Matrix3d::Identity());
  state_.set_velocity_covariance(velocity_cov_val_
                                 * Eigen::Matrix3d::Identity());
  state_.set_position_covariance(position_cov_val_
                                 * Eigen::Matrix3d::Identity());

  std::cout << "Robot's state mean is initialized to: \n";
  std::cout << this->get_state().get_X() << std::endl;
  std::cout << "Robot's state covariance is initialized to: \n";
  std::cout << this->get_state().get_P() << std::endl;

  // Push the initial state to the queue
  robot_state_queue_mutex_ptr_.get()->lock();
  robot_state_queue_ptr_.get()->push(std::make_shared<RobotState>(state_));
  robot_state_queue_mutex_ptr_.get()->unlock();

  // Set enabled flag
  enabled_ = true;
}

void InekfEstimator::clear() {
  // Clear IMU filter if it exists
  if (imu_filter_) {
    imu_filter_.get()->clear();
  }

  // Clear propagation
  propagation_.get()->clear();

  // Clear corrections
  for (auto& correction : corrections_) {
    correction.get()->clear();
  }

  // Clear the state
  state_.clear();

  // Clear state queue
  robot_state_queue_mutex_ptr_.get()->lock();
  while (!robot_state_queue_ptr_.get()->empty()) {
    robot_state_queue_ptr_.get()->pop();
  }
  robot_state_queue_mutex_ptr_.get()->unlock();

  new_pose_ready_ = false;
  // Must init the state first before enabling the filter
  enabled_ = false;

  // Clear current log queue and open a new file for logging
  robot_state_log_queue_mutex_.lock();
  while (!robot_state_log_queue_ptr_.get()->empty()) {
    robot_state_log_queue_ptr_.get()->pop();
  }
  robot_state_log_queue_mutex_.unlock();
  if (enable_pose_logger_) {
    outfile_.close();
    outfile_.open(pose_log_file_ + '_' + std::to_string(++init_count_)
                  + ".txt");
    outfile_.precision(dbl::max_digits10);
    vel_outfile_.close();
    vel_outfile_.open(vel_log_file_ + '_' + std::to_string(++init_count_)
                      + ".txt");
    vel_outfile_.precision(dbl::max_digits10);
    vel_body_outfile_.close();
    vel_body_outfile_.open(vel_log_file_ + "_body_frame_"
                           + std::to_string(++init_count_) + ".txt");
    vel_body_outfile_.precision(dbl::max_digits10);
  }
}

void InekfEstimator::SlipEstimatorStep() {
  Eigen::MatrixXd P = state_.get_P();
  Eigen::Matrix3d R = state_.get_rotation();
  auto disturbance_est = state_.get_aug_state(state_.dimX() - 1);

  Eigen::MatrixXd G;
  G.conservativeResize(3, 6);
  G.block(0, 0, 3, 3) = lie_group::skew(disturbance_est);
  G.block(0, 3, 3, 3) = -Eigen::MatrixXd::Identity(3, 3);

  Eigen::MatrixXd Cov;
  Cov.conservativeResize(6, 6);
  Cov.block(0, 0, 3, 3) = P.block(0, 0, 3, 3);
  Cov.block(0, 3, 3, 3) = P.block(0, 9, 3, 3);
  Cov.block(3, 0, 3, 3) = P.block(9, 0, 3, 3);
  Cov.block(3, 3, 3, 3) = P.block(9, 9, 3, 3);

  Eigen::Matrix3d Sigma = G * Cov * G.transpose();
  Eigen::Matrix3d Sigma1 = 0.001 * Eigen::MatrixXd::Identity(3, 3);

  double chi
      = (disturbance_est).transpose() * (Sigma1).inverse() * (disturbance_est);

  if (chi > 4.642) {
    state_.set_slip_flag(1);
  } else {
    state_.set_slip_flag(0);
  }
}

}    // namespace estimator
