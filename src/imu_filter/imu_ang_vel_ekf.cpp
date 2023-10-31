/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   gyro_filter.cpp
 *  @author Tzu-Yuan Lin, Tingjun Li
 *  @brief
 *
 *  @date   May 16, 2023
 **/

#include "drift/imu_filter/imu_ang_vel_ekf.h"


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
  cout << "Loading gyro filter config from " << yaml_filepath << endl;
  YAML::Node config_ = YAML::LoadFile(yaml_filepath);


  // Set the imu to body rotation (bring imu measurements to body frame)
  const std::vector<double> quat_imu2body
      = config_["settings"]["rotation_imu2body"]
            ? config_["settings"]["rotation_imu2body"].as<std::vector<double>>()
            : std::vector<double>({1, 0, 0, 0});

  bool flat_ground = config_["settings"]["flat_ground"]
                         ? config_["settings"]["flat_ground"].as<bool>()
                         : false;

  Eigen::Quaternion<double> quarternion_imu2body(
      quat_imu2body[0], quat_imu2body[1], quat_imu2body[2], quat_imu2body[3]);
  R_imu2body_inverse_ = quarternion_imu2body.toRotationMatrix().transpose();

  // Set time threshold between propagation and correction
  t_thres_ = config_["settings"]["t_threshold"]
                 ? config_["settings"]["t_threshold"].as<double>()
                 : 0.03;

  // Set correction method:
  correction_method_ = CORRECTION_METHOD(
      config_["settings"]["correction_method"]
          ? config_["settings"]["correction_method"].as<int>()
          : 0);

  // Set the initial bias
  static_bias_initialization_
      = config_["settings"]["static_bias_initialization"]
            ? config_["settings"]["static_bias_initialization"].as<bool>()
            : false;


  std::cout << "static bias init: " << static_bias_initialization_ << std::endl;

  if (static_bias_initialization_ == false) {
    std::vector<double> ang_vel_bias
        = config_["priors"]["ang_vel_bias"]
              ? config_["priors"]["ang_vel_bias"].as<std::vector<double>>()
              : std::vector<double>({0, 0, 0});
    ang_vel_and_bias_est_(3) = ang_vel_bias[0];
    ang_vel_and_bias_est_(4) = ang_vel_bias[1];
    ang_vel_and_bias_est_(5) = ang_vel_bias[2];

    std::cout
        << "Static bias initialization is set to false for the gyro filter. \n "
           "The biases in the gyro filter are initialized using prior as: ["
        << ang_vel_and_bias_est_(3) << ", " << ang_vel_and_bias_est_(4) << ", "
        << ang_vel_and_bias_est_(5) << "]" << std::endl;
  }

  // Set the noise parameters
  double ang_vel_std = config_["noises"]["ang_vel_std"]
                           ? config_["noises"]["ang_vel_std"].as<double>()
                           : 0.01;

  double ang_vel_bias_std
      = config_["noises"]["ang_vel_bias_std"]
            ? config_["noises"]["ang_vel_bias_std"].as<double>()
            : 0.01;
  double filter_std = config_["noises"]["filter_std"]
                          ? config_["noises"]["filter_std"].as<double>()
                          : 0.01;
  double filter_bias_std
      = config_["noises"]["filter_bias_std"]
            ? config_["noises"]["filter_bias_std"].as<double>()
            : 0.01;
  double imu_meas_noise_std
      = config_["noises"]["imu_meas_noise_std"]
            ? config_["noises"]["imu_meas_noise_std"].as<double>()
            : 0.1;
  double encoder_meas_noise_std
      = config_["noises"]["encoder_meas_noise_std"]
            ? config_["noises"]["encoder_meas_noise_std"].as<double>()
            : 0.1;
  ang_vel_and_bias_P_ = Eigen::MatrixXd::Zero(6, 6);
  ang_vel_and_bias_P_.block<3, 3>(0, 0)
      = ang_vel_std * ang_vel_std * Eigen::Matrix3d::Identity();
  ang_vel_and_bias_P_.block<3, 3>(3, 3)
      = ang_vel_bias_std * ang_vel_bias_std * Eigen::Matrix3d::Identity();

  ang_vel_and_bias_Q_ = Eigen::MatrixXd::Zero(6, 6);
  ang_vel_and_bias_Q_.block<3, 3>(0, 0)
      = filter_std * filter_std
        * Eigen::Matrix3d::Identity();    // Process noise
  ang_vel_and_bias_Q_.block<3, 3>(3, 3)
      = filter_bias_std * filter_bias_std
        * Eigen::Matrix3d::Identity();    // Process noise for bias

  // IMU measurement ang vel noise
  ang_vel_imu_R_
      = imu_meas_noise_std * imu_meas_noise_std * Eigen::Matrix3d::Identity();

  // Encoder measurement ang vel noise
  ang_vel_enc_R_ = encoder_meas_noise_std * encoder_meas_noise_std
                   * Eigen::Matrix3d::Identity();

  // Set the parameters for the angular velocity filter
  if (flat_ground == true) {
    H_imu_.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3);
    H_imu_.block<3, 3>(0, 3) = Eigen::MatrixXd::Identity(3, 3);
    H_enc_.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3);
    H_enc_.block<3, 3>(0, 3) = Eigen::MatrixXd::Zero(3, 3);
  } else {
    // 1D imu filter
    /// TODO: Add a switch for 1D or 3D filter
    H_imu_ = Eigen::MatrixXd::Zero(3, 6);
    H_enc_ = Eigen::MatrixXd::Zero(3, 6);
    H_imu_(2, 2) = 1;
    H_imu_(2, 5) = 1;
    H_enc_(2, 2) = 1;
  }

  // Temp logger
  std::string imu_ang_vel_log_file
      = "/home/justin/code/drift/log/imu_ang_vel_log.txt";
  imu_ang_vel_outfile_.open(imu_ang_vel_log_file);
  imu_ang_vel_outfile_.precision(dbl::max_digits10);

  std::string encoder_ang_vel_log_file
      = "/home/justin/code/drift/log/encoder_ang_vel_log.txt";
  encoder_ang_vel_outfile_.open(encoder_ang_vel_log_file);
  encoder_ang_vel_outfile_.precision(dbl::max_digits10);

  std::string filtered_ang_vel_log_file
      = "/home/justin/code/drift/log/filtered_ang_vel_log.txt";
  filtered_ang_vel_outfile_.open(filtered_ang_vel_log_file);
  filtered_ang_vel_outfile_.precision(dbl::max_digits10);
}

ImuAngVelEKF::ImuAngVelEKF(const std::string& yaml_filepath)
    : filtered_imu_data_buffer_ptr_(new IMUQueue),
      filtered_imu_data_buffer_mutex_ptr_(new std::mutex) {
  // Load configs
  cout << "Loading gyro filter config from " << yaml_filepath << endl;
  YAML::Node config_ = YAML::LoadFile(yaml_filepath);


  // Set the imu to body rotation (bring imu measurements to body frame)
  const std::vector<double> quat_imu2body
      = config_["settings"]["rotation_imu2body"]
            ? config_["settings"]["rotation_imu2body"].as<std::vector<double>>()
            : std::vector<double>({1, 0, 0, 0});

  bool flat_ground = config_["settings"]["flat_ground"]
                         ? config_["settings"]["flat_ground"].as<bool>()
                         : false;

  Eigen::Quaternion<double> quarternion_imu2body(
      quat_imu2body[0], quat_imu2body[1], quat_imu2body[2], quat_imu2body[3]);
  R_imu2body_inverse_ = quarternion_imu2body.toRotationMatrix().transpose();

  // Set time threshold between propagation and correction
  t_thres_ = config_["settings"]["t_threshold"]
                 ? config_["settings"]["t_threshold"].as<double>()
                 : 0.03;

  // Set correction method:
  correction_method_ = CORRECTION_METHOD(
      config_["settings"]["correction_method"]
          ? config_["settings"]["correction_method"].as<int>()
          : 0);

  init_bias_size_ = config_["settings"]["init_bias_size"]
                        ? config_["settings"]["init_bias_size"].as<int>()
                        : 250;

  // Set the noise parameters
  double ang_vel_std = config_["noises"]["ang_vel_std"]
                           ? config_["noises"]["ang_vel_std"].as<double>()
                           : 0.01;

  double ang_vel_bias_std
      = config_["noises"]["ang_vel_bias_std"]
            ? config_["noises"]["ang_vel_bias_std"].as<double>()
            : 0.01;
  double filter_std = config_["noises"]["filter_std"]
                          ? config_["noises"]["filter_std"].as<double>()
                          : 0.01;
  double filter_bias_std
      = config_["noises"]["filter_bias_std"]
            ? config_["noises"]["filter_bias_std"].as<double>()
            : 0.01;
  double imu_meas_noise_std
      = config_["noises"]["imu_meas_noise_std"]
            ? config_["noises"]["imu_meas_noise_std"].as<double>()
            : 0.1;
  double encoder_meas_noise_std
      = config_["noises"]["encoder_meas_noise_std"]
            ? config_["noises"]["encoder_meas_noise_std"].as<double>()
            : 0.1;

  ang_vel_and_bias_P_ = Eigen::MatrixXd::Zero(6, 6);
  ang_vel_and_bias_P_.block<3, 3>(0, 0)
      = ang_vel_std * ang_vel_std * Eigen::Matrix3d::Identity();
  ang_vel_and_bias_P_.block<3, 3>(3, 3)
      = ang_vel_bias_std * ang_vel_bias_std * Eigen::Matrix3d::Identity();


  ang_vel_and_bias_Q_ = Eigen::MatrixXd::Zero(6, 6);
  ang_vel_and_bias_Q_.block<3, 3>(0, 0)
      = filter_std * filter_std
        * Eigen::Matrix3d::Identity();    // Process noise
  ang_vel_and_bias_Q_.block<3, 3>(3, 3)
      = filter_bias_std * filter_bias_std
        * Eigen::Matrix3d::Identity();    // Process noise for bias

  // IMU measurement ang vel noise
  ang_vel_imu_R_
      = imu_meas_noise_std * imu_meas_noise_std * Eigen::Matrix3d::Identity();

  // Encoder measurement ang vel noise
  ang_vel_enc_R_ = encoder_meas_noise_std * encoder_meas_noise_std
                   * Eigen::Matrix3d::Identity();

  // Set the parameters for the angular velocity filter
  if (flat_ground == true) {
    H_imu_.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3);
    H_imu_.block<3, 3>(0, 3) = Eigen::MatrixXd::Identity(3, 3);
    H_enc_.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3);
    H_enc_.block<3, 3>(0, 3) = Eigen::MatrixXd::Zero(3, 3);
  } else {
    // 1D imu filter
    H_imu_ = Eigen::MatrixXd::Zero(3, 6);
    H_enc_ = Eigen::MatrixXd::Zero(3, 6);
    H_imu_(2, 2) = 1;
    H_imu_(2, 5) = 1;
    H_enc_(2, 2) = 1;
  }


  // Set the initial bias
  static_bias_initialization_
      = config_["settings"]["static_bias_initialization"]
            ? config_["settings"]["static_bias_initialization"].as<bool>()
            : false;

  std::cout << "static bias init: " << static_bias_initialization_ << std::endl;

  if (static_bias_initialization_ == false) {
    std::vector<double> ang_vel_bias
        = config_["priors"]["ang_vel_bias"]
              ? config_["priors"]["ang_vel_bias"].as<std::vector<double>>()
              : std::vector<double>({0, 0, 0});
    ang_vel_and_bias_est_(3) = ang_vel_bias[0];
    ang_vel_and_bias_est_(4) = ang_vel_bias[1];
    ang_vel_and_bias_est_(5) = ang_vel_bias[2];

    std::cout
        << "Static bias initialization is set to false for the gyro filter. \n "
           "The biases in the gyro filter are initialized using prior as: ["
        << ang_vel_and_bias_est_(3) << ", " << ang_vel_and_bias_est_(4) << ", "
        << ang_vel_and_bias_est_(5) << "]" << std::endl;
  }


  // Temp logger
  std::string imu_propagate_input_file
      = "/home/justin/code/drift/log/imu_propagate_input_log.txt";
  imu_propagate_input_outfile_.open(imu_propagate_input_file);
  imu_propagate_input_outfile_.precision(dbl::max_digits10);

  std::string imu_propagate_file
      = "/home/justin/code/drift/log/imu_propagate_log_.txt";
  imu_propagate_outfile_.open(imu_propagate_file);
  imu_propagate_outfile_.precision(dbl::max_digits10);

  std::string imu_ang_vel_log_file
      = "/home/justin/code/drift/log/imu_correction_input_log.txt";
  imu_ang_vel_outfile_.open(imu_ang_vel_log_file);
  imu_ang_vel_outfile_.precision(dbl::max_digits10);

  std::string encoder_ang_vel_log_file
      = "/home/justin/code/drift/log/encoder_input_log.txt";
  encoder_ang_vel_outfile_.open(encoder_ang_vel_log_file);
  encoder_ang_vel_outfile_.precision(dbl::max_digits10);

  std::string filtered_ang_vel_log_file
      = "/home/justin/code/drift/log/filtered_output_log.txt";
  filtered_ang_vel_outfile_.open(filtered_ang_vel_log_file);
  filtered_ang_vel_outfile_.precision(dbl::max_digits10);
}

// IMU filter destructor
ImuAngVelEKF::~ImuAngVelEKF() {
  stop_thread_ = true;
  imu_filter_thread_.join();
  imu_propagate_input_outfile_.close();
  imu_propagate_outfile_.close();
  imu_ang_vel_outfile_.close();
  encoder_ang_vel_outfile_.close();
  filtered_ang_vel_outfile_.close();
}

void ImuAngVelEKF::add_gyro_propagate(
    IMUQueuePtr imu_data_buffer_ptr,
    std::shared_ptr<std::mutex> imu_data_buffer_mutex_ptr) {
  gyro_prop_data_buffer_ptr_ = imu_data_buffer_ptr;
  gyro_prop_data_buffer_mutex_ptr_ = imu_data_buffer_mutex_ptr;
  propagation_method_ = GYRO;
}

void ImuAngVelEKF::add_imu_correction(
    IMUQueuePtr imu_data_buffer_ptr,
    std::shared_ptr<std::mutex> imu_data_buffer_mutex_ptr) {
  num_imu_++;
  if (num_imu_ == 1) {
    // For single imu version:
    imu_data_buffer_ptr_ = imu_data_buffer_ptr;
    imu_data_buffer_mutex_ptr_ = imu_data_buffer_mutex_ptr;
  } else {
    // For all other versions:
    imu_data_buffer_ptrs_.push_back(imu_data_buffer_ptr);
    imu_data_buffer_mutex_ptrs_.push_back(imu_data_buffer_mutex_ptr);
  }
}

void ImuAngVelEKF::add_ang_vel_correction(
    AngularVelocityQueuePtr ang_vel_data_buffer_ptr,
    std::shared_ptr<std::mutex> ang_vel_data_buffer_mutex_ptr) {
  num_ang_vel_++;
  ang_vel_data_buffer_ptr_ = ang_vel_data_buffer_ptr;
  ang_vel_data_buffer_mutex_ptr_ = ang_vel_data_buffer_mutex_ptr;
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
  if (filter_initialized_) {
    switch (propagation_method_) {
      case GYRO:
        GyroPropagate();
        break;
      case RANDOM_WALK:
        RandomWalkPropagate();
        break;
    }

    switch (correction_method_) {
      //
      case SINGLE_IMU_PLUS_ANG_VEL:
        SingleImuAngVelCorrection();
        break;
      case SINGLE_IMU:
        SingleImuCorrection();
        break;
      case MULTI_IMU:
        MultiImuCorrection();
        break;
      case ANG_VEL:
        AngVelCorrection();
        break;
      default:
        std::cerr << "Please name a correction enumeration" << std::endl;
        break;
    }
  } else {
    if (num_imu_ == 0 && num_ang_vel_ == 1) {
      correction_method_ = ANG_VEL;
    } else if (num_imu_ == 1 && num_ang_vel_ == 0) {
      correction_method_ = SINGLE_IMU;
    } else if (num_imu_ == 1 && num_ang_vel_ == 1) {
      correction_method_ = SINGLE_IMU_PLUS_ANG_VEL;
    } else if (num_imu_ > 1 && num_ang_vel_ == 0) {
      correction_method_ = MULTI_IMU;
    } else {
      std::cerr << "Incorrect correction method in the gyro filter. "
                << std::endl;
      std::cerr << "Number of imu correction: " << num_imu_
                << ", number of ang vel correction: " << num_ang_vel_
                << std::endl;
    }

    // std::cout << "initializing filter" << std::endl;
    // Initialize filter
    InitializeFilter();
  }
  // LogInputIMU();
}

void ImuAngVelEKF::LogInputIMU() {
  ImuMeasurementPtr imu1_measurement;
  imu_data_buffer_mutex_ptr_.get()->lock();
  if (imu_data_buffer_ptr_->empty()) {
    imu_data_buffer_mutex_ptr_.get()->unlock();
  } else {
    imu1_measurement = imu_data_buffer_ptr_.get()->front();
    imu_data_buffer_ptr_.get()->pop();
    imu_data_buffer_mutex_ptr_.get()->unlock();
    imu_propagate_input_outfile_
        << imu1_measurement->get_time() << ","
        << imu1_measurement->get_angular_velocity()(0) << ","
        << imu1_measurement->get_angular_velocity()(1) << ","
        << imu1_measurement->get_angular_velocity()(2) << std::endl
        << std::flush;
  }

  ImuMeasurementPtr imu0_measurement;
  gyro_prop_data_buffer_mutex_ptr_.get()->lock();
  if (gyro_prop_data_buffer_ptr_->empty()) {
    gyro_prop_data_buffer_mutex_ptr_.get()->unlock();
  } else {
    imu0_measurement = gyro_prop_data_buffer_ptr_->front();
    gyro_prop_data_buffer_ptr_->pop();
    gyro_prop_data_buffer_mutex_ptr_.get()->unlock();
    imu_ang_vel_outfile_ << imu0_measurement->get_time() << ","
                         << imu0_measurement->get_angular_velocity()(0) << ","
                         << imu0_measurement->get_angular_velocity()(1) << ","
                         << imu0_measurement->get_angular_velocity()(2)
                         << std::endl
                         << std::flush;
  }
}

void ImuAngVelEKF::InitializeFilter() {
  /// TODO: Currently only support single imu correction

  // std::cout << "bias_init set to " << static_bias_initialization_ <<
  // std::endl;
  if (bias_init_vec_.size() < init_bias_size_ && static_bias_initialization_) {
    imu_data_buffer_mutex_ptr_.get()->lock();
    if (imu_data_buffer_ptr_->empty()) {
      imu_data_buffer_mutex_ptr_.get()->unlock();
      return;
    }
    ImuMeasurementPtr imu1_measurement = imu_data_buffer_ptr_.get()->front();
    imu_data_buffer_ptr_.get()->pop();
    imu_data_buffer_mutex_ptr_.get()->unlock();
    bias_init_vec_.push_back(imu1_measurement->get_angular_velocity());

    if (propagation_method_ == GYRO) {
      gyro_prop_data_buffer_mutex_ptr_.get()->lock();
      if (gyro_prop_data_buffer_ptr_->empty()) {
        gyro_prop_data_buffer_mutex_ptr_.get()->unlock();
        return;
      }

      ImuMeasurementPtr imu0_measurement = gyro_prop_data_buffer_ptr_->front();
      gyro_prop_data_buffer_ptr_->pop();
      gyro_prop_data_buffer_mutex_ptr_.get()->unlock();

      prev_ang_vel_measurement_ = imu0_measurement->get_angular_velocity();
    }
  } else if (static_bias_initialization_) {
    Eigen::Matrix<double, 3, 1> avg = Eigen::Matrix<double, 3, 1>::Zero();
    for (int i = 0; i < bias_init_vec_.size(); ++i) {
      avg = (avg + bias_init_vec_[i]).eval();
    }
    avg = (avg / bias_init_vec_.size()).eval();
    std::cout << "Biases in the gyro filter are initialized to: "
              << avg.transpose() << std::endl;

    ang_vel_and_bias_est_.head(3) = Eigen::Matrix<double, 3, 1>::Zero();
    ang_vel_and_bias_est_.tail(3) = avg;

    // Clear the queue
    if (propagation_method_ == GYRO) {
      std::queue<ImuMeasurementPtr> empty;
      gyro_prop_data_buffer_mutex_ptr_.get()->lock();
      std::swap(*gyro_prop_data_buffer_ptr_, empty);
      gyro_prop_data_buffer_mutex_ptr_.get()->unlock();
    }

    if (correction_method_ == SINGLE_IMU_PLUS_ANG_VEL
        || correction_method_ == SINGLE_IMU) {
      std::queue<ImuMeasurementPtr> empty;
      imu_data_buffer_mutex_ptr_.get()->lock();
      std::swap(*imu_data_buffer_ptr_, empty);
      imu_data_buffer_mutex_ptr_.get()->unlock();
    } else if (correction_method_ == MULTI_IMU) {
      for (int i = 0; i < imu_data_buffer_mutex_ptrs_.size(); i++) {
        std::queue<ImuMeasurementPtr> empty;
        imu_data_buffer_mutex_ptrs_[i].get()->lock();
        std::swap(*imu_data_buffer_ptrs_[i], empty);
        imu_data_buffer_mutex_ptrs_[i].get()->unlock();
      }
    }

    if (correction_method_ == SINGLE_IMU_PLUS_ANG_VEL
        || correction_method_ == ANG_VEL) {
      std::queue<AngularVelocityMeasurementPtr> empty;
      ang_vel_data_buffer_mutex_ptr_.get()->lock();
      std::swap(*ang_vel_data_buffer_ptr_, empty);
      ang_vel_data_buffer_mutex_ptr_.get()->unlock();
    }
    filter_initialized_ = true;
  } else {
    filter_initialized_ = true;
  }
}

void ImuAngVelEKF::SingleImuCorrection() {
  // Bias corrected IMU measurements
  imu_data_buffer_mutex_ptr_.get()->lock();
  if (imu_data_buffer_ptr_->empty()) {
    imu_data_buffer_mutex_ptr_.get()->unlock();
    return;
  }

  ImuMeasurementPtr imu_measurement = imu_data_buffer_ptr_.get()->front();

  // Only perform correction step if with data prior to the propagation time
  // and the time gap is smaller than a threshold
  /// TODO: double check if we need absolute here?
  // if (last_propagate_time_ == -1
  // || last_propagate_time_ - imu_measurement->get_time() <= t_thres_) {
  if (last_propagate_time_ - imu_measurement->get_time() <= t_thres_
      && last_propagate_time_ >= imu_measurement->get_time()) {
    imu_data_buffer_ptr_.get()->pop();
    imu_data_buffer_mutex_ptr_.get()->unlock();

    auto fused_ang_imu = AngVelFilterCorrectIMU(imu_measurement);
    filtered_imu_data_buffer_mutex_ptr_.get()->lock();
    filtered_imu_data_buffer_ptr_->push(fused_ang_imu);
    filtered_imu_data_buffer_mutex_ptr_.get()->unlock();
  } else {
    imu_data_buffer_mutex_ptr_.get()->unlock();
  }
}

void ImuAngVelEKF::MultiImuCorrection() {
  ImuMeasurementPtr imu_measurement = nullptr;
  ImuMeasurementPtr fused_ang_imu = nullptr;

  // Bias corrected IMU measurements
  for (int i = 0; i < imu_data_buffer_mutex_ptrs_.size(); i++) {
    imu_data_buffer_mutex_ptrs_[i].get()->lock();
    if (imu_data_buffer_ptrs_[i]->empty()) {
      imu_data_buffer_mutex_ptrs_[i].get()->unlock();
      continue;
    }

    imu_measurement = imu_data_buffer_ptrs_[i].get()->front();
    imu_data_buffer_ptrs_[i].get()->pop();
    imu_data_buffer_mutex_ptrs_[i].get()->unlock();

    // Only perform correction step if with data prior to the propagation time
    // and the time gap is smaller than a threshold
    if (last_propagate_time_ == -1
        || last_propagate_time_ - imu_measurement->get_time() <= t_thres_) {
      fused_ang_imu = AngVelFilterCorrectIMU(imu_measurement);
    }

    if (fused_ang_imu) {
      filtered_imu_data_buffer_mutex_ptr_.get()->lock();
      filtered_imu_data_buffer_ptr_->push(fused_ang_imu);
      filtered_imu_data_buffer_mutex_ptr_.get()->unlock();
    }
  }
}

void ImuAngVelEKF::SingleImuAngVelCorrection() {
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
  if (imu_measurement) {
    latest_imu_measurement_ = imu_measurement;
    if (last_propagate_time_ == -1
        || last_propagate_time_ - imu_measurement->get_time() <= t_thres_) {
      auto fused_ang_imu = AngVelFilterCorrectIMU(imu_measurement);
      filtered_imu_data_buffer_mutex_ptr_.get()->lock();
      filtered_imu_data_buffer_ptr_->push(fused_ang_imu);
      filtered_imu_data_buffer_mutex_ptr_.get()->unlock();
    }

    imu_data_buffer_mutex_ptr_.get()->lock();
    imu_data_buffer_ptr_.get()->pop();
    imu_data_buffer_mutex_ptr_.get()->unlock();
    return;
  }

  if (ang_vel_measurement && !latest_imu_measurement_) {
    ang_vel_data_buffer_mutex_ptr_.get()->lock();
    ang_vel_data_buffer_ptr_.get()->pop();
    ang_vel_data_buffer_mutex_ptr_.get()->unlock();
    return;
  }

  // only execute the following code if we have a latest_imu_measurement_
  if (ang_vel_measurement && latest_imu_measurement_) {
    if (last_propagate_time_ == -1
        || last_propagate_time_ - ang_vel_measurement->get_time() <= t_thres_) {
      auto fused_ang_imu = AngVelFilterCorrectEncoder(latest_imu_measurement_,
                                                      ang_vel_measurement);
      filtered_imu_data_buffer_mutex_ptr_.get()->lock();
      filtered_imu_data_buffer_ptr_->push(fused_ang_imu);
      filtered_imu_data_buffer_mutex_ptr_.get()->unlock();
    }

    ang_vel_data_buffer_mutex_ptr_.get()->lock();
    ang_vel_data_buffer_ptr_->pop();
    ang_vel_data_buffer_mutex_ptr_.get()->unlock();
    return;
  }
}

void ImuAngVelEKF::AngVelCorrection() {
  // Angular velocity measurements
  ang_vel_data_buffer_mutex_ptr_.get()->lock();
  if (ang_vel_data_buffer_ptr_->empty()) {
    ang_vel_data_buffer_mutex_ptr_.get()->unlock();
    return;
  }

  AngularVelocityMeasurementPtr ang_vel_measurement
      = ang_vel_data_buffer_ptr_->front();
  ang_vel_data_buffer_mutex_ptr_.get()->unlock();


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
  // if (enable_gyro_propagate_) {
  //   this->GyroPropagate();
  // } else {
  //   this->RandomWalkPropagate();
  // }
}

void ImuAngVelEKF::GyroPropagate() {
  // Take Gyro data:
  gyro_prop_data_buffer_mutex_ptr_.get()->lock();
  if (gyro_prop_data_buffer_ptr_->empty()) {
    gyro_prop_data_buffer_mutex_ptr_.get()->unlock();
    return;
  }

  ImuMeasurementPtr imu_measurement = gyro_prop_data_buffer_ptr_->front();
  gyro_prop_data_buffer_ptr_->pop();
  gyro_prop_data_buffer_mutex_ptr_.get()->unlock();

  last_propagate_time_ = imu_measurement->get_time();

  if (correction_method_ == ANG_VEL) {
    latest_imu_measurement_ = imu_measurement;
  }

  // Use gyro measurement data to perform propagation
  // w_(k+1) = w_(t+1) - w_t + w_k
  Eigen::VectorXd measurement_diff = Eigen::VectorXd::Zero(6);
  measurement_diff.block<3, 1>(0, 0)
      = imu_measurement->get_angular_velocity() - prev_ang_vel_measurement_;
  prev_ang_vel_measurement_ = imu_measurement->get_angular_velocity();

  ang_vel_and_bias_est_ = measurement_diff + ang_vel_and_bias_est_;

  // Covariance propagation
  ang_vel_and_bias_P_ = ang_vel_and_bias_P_ + ang_vel_and_bias_Q_;


  Eigen::VectorXd imu_ang_vel = imu_measurement->get_angular_velocity();
  imu_propagate_input_outfile_ << imu_measurement->get_time() << ","
                               << imu_ang_vel(0) << "," << imu_ang_vel(1) << ","
                               << imu_ang_vel(2) << std::endl
                               << std::flush;

  imu_propagate_outfile_ << imu_measurement->get_time() << ","
                         << ang_vel_and_bias_est_(0) << ","
                         << ang_vel_and_bias_est_(1) << ","
                         << ang_vel_and_bias_est_(2) << ","
                         << ang_vel_and_bias_est_(3) << ","
                         << ang_vel_and_bias_est_(4) << ","
                         << ang_vel_and_bias_est_(5) << std::endl
                         << std::flush;
}

void ImuAngVelEKF::RandomWalkPropagate() {
  // Random walk
  // X = X, so we don't need to do anything for mean propagation

  // Covariance propagation
  ang_vel_and_bias_P_ = ang_vel_and_bias_P_ + ang_vel_and_bias_Q_;
}

ImuMeasurementPtr ImuAngVelEKF::AngVelFilterCorrectIMU(
    const ImuMeasurementPtr& imu_measurement) {
  Eigen::VectorXd z = imu_measurement->get_angular_velocity()
                      - H_imu_ * ang_vel_and_bias_est_;
  auto S_inv
      = (H_imu_ * ang_vel_and_bias_P_ * H_imu_.transpose() + ang_vel_imu_R_)
            .inverse();
  auto K = ang_vel_and_bias_P_ * H_imu_.transpose() * S_inv;
  ang_vel_and_bias_est_ = ang_vel_and_bias_est_ + K * z;
  ang_vel_and_bias_P_
      = (Eigen::MatrixXd::Identity(6, 6) - K * H_imu_) * ang_vel_and_bias_P_;

  ImuMeasurementPtr imu_measurement_corrected
      = std::make_shared<ImuMeasurement<double>>(*imu_measurement);
  imu_measurement_corrected->set_angular_velocity(ang_vel_and_bias_est_(0),
                                                  ang_vel_and_bias_est_(1),
                                                  ang_vel_and_bias_est_(2));
  imu_measurement_corrected->set_time(imu_measurement->get_time());

  Eigen::VectorXd imu_ang_vel = imu_measurement->get_angular_velocity();
  imu_ang_vel_outfile_ << imu_measurement->get_time() << "," << imu_ang_vel(0)
                       << "," << imu_ang_vel(1) << "," << imu_ang_vel(2)
                       << std::endl
                       << std::flush;

  filtered_ang_vel_outfile_
      << imu_measurement_corrected->get_time() << ","
      << ang_vel_and_bias_est_(0) << "," << ang_vel_and_bias_est_(1) << ","
      << ang_vel_and_bias_est_(2) << "," << ang_vel_and_bias_est_(3) << ","
      << ang_vel_and_bias_est_(4) << "," << ang_vel_and_bias_est_(5)
      << std::endl
      << std::flush;

  return imu_measurement_corrected;
}

ImuMeasurementPtr ImuAngVelEKF::AngVelFilterCorrectEncoder(
    const ImuMeasurementPtr& imu_measurement,
    const AngularVelocityMeasurementPtr& ang_vel_measurement) {
  Eigen::VectorXd z
      = R_imu2body_inverse_ * ang_vel_measurement->get_angular_velocity()
        - H_enc_ * ang_vel_and_bias_est_;

  auto S_inv
      = (H_enc_ * ang_vel_and_bias_P_ * H_enc_.transpose() + ang_vel_enc_R_)
            .inverse();
  auto K = ang_vel_and_bias_P_ * H_enc_.transpose() * S_inv;
  ang_vel_and_bias_est_ = ang_vel_and_bias_est_ + K * z;
  ang_vel_and_bias_P_
      = (Eigen::MatrixXd::Identity(6, 6) - K * H_enc_) * ang_vel_and_bias_P_;

  ImuMeasurementPtr imu_measurement_corrected
      = std::make_shared<ImuMeasurement<double>>(*imu_measurement);
  imu_measurement_corrected->set_angular_velocity(ang_vel_and_bias_est_(0),
                                                  ang_vel_and_bias_est_(1),
                                                  ang_vel_and_bias_est_(2));
  imu_measurement_corrected->set_time(ang_vel_measurement->get_time());

  Eigen::VectorXd enc_ang_vel
      = R_imu2body_inverse_ * ang_vel_measurement->get_angular_velocity();

  encoder_ang_vel_outfile_ << ang_vel_measurement->get_time() << ","
                           << enc_ang_vel(0) << "," << enc_ang_vel(1) << ","
                           << enc_ang_vel(2) << std::endl
                           << std::flush;

  filtered_ang_vel_outfile_
      << imu_measurement_corrected->get_time() << ","
      << ang_vel_and_bias_est_(0) << "," << ang_vel_and_bias_est_(1) << ","
      << ang_vel_and_bias_est_(2) << "," << ang_vel_and_bias_est_(3) << ","
      << ang_vel_and_bias_est_(4) << "," << ang_vel_and_bias_est_(5)
      << std::endl
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

std::pair<IMUQueuePtr, std::shared_ptr<std::mutex>>
ImuAngVelEKF::get_filtered_imu_data_buffer_and_mutex_ptr() {
  return {filtered_imu_data_buffer_ptr_, filtered_imu_data_buffer_mutex_ptr_};
};

void ImuAngVelEKF::clear() {
  // Clear Imu data buffer
  imu_data_buffer_mutex_ptr_.get()->lock();
  while (!imu_data_buffer_ptr_->empty()) {
    imu_data_buffer_ptr_->pop();
  }
  imu_data_buffer_mutex_ptr_.get()->unlock();

  // Clear angular velocity data buffer
  ang_vel_data_buffer_mutex_ptr_.get()->lock();
  while (!ang_vel_data_buffer_ptr_->empty()) {
    ang_vel_data_buffer_ptr_->pop();
  }
  ang_vel_data_buffer_mutex_ptr_.get()->unlock();

  // Clear filtered_imu_data_buffer
  filtered_imu_data_buffer_mutex_ptr_.get()->lock();
  while (!filtered_imu_data_buffer_ptr_->empty()) {
    filtered_imu_data_buffer_ptr_->pop();
  }
  filtered_imu_data_buffer_mutex_ptr_.get()->unlock();
}
}    // namespace imu_filter