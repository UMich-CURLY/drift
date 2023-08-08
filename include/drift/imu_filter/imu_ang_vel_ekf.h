/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * --------------------------------------------------------------------------
 */

/**
 *  @file   imu_ang_vel_ekf.h
 *  @author Tzu-Yuan Lin, Tingjun Li
 *
 *  @date   May 16, 2023
 **/

#ifndef IMU_FILTER_IMU_ANG_VEL_EKF_H
#define IMU_FILTER_IMU_ANG_VEL_EKF_H

#include <atomic>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <thread>
#include <vector>


#include "drift/filter/base_propagation.h"
#include "drift/filter/inekf/inekf.h"
#include "drift/math/lie_group.h"
#include "drift/measurement/angular_velocity.h"
#include "drift/measurement/imu.h"
#include "drift/utils/type_def.h"

using namespace math;
using namespace state;
using namespace measurement;


namespace imu_filter {

enum PROPAGATION_METHOD {
  RANDOM_WALK = 0,
  GYRO = 1,
};

enum CORRECTION_METHOD {
  SINGLE_IMU_PLUS_ANG_VEL = 0,
  SINGLE_IMU = 1,
  MULTI_IMU = 2,
  ANG_VEL = 3,
};

/**
 * @class ImuAngVelEKF
 */
class ImuAngVelEKF {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name Constructors
  /// @{
  // ======================================================================
  /**
   * @brief Constructor for the propagation class
   *
   * @param[in] imu_data_buffer_ptr: Pointer to the buffer of IMU sensor data.
   * @param[in] imu_data_buffer_mutex_ptr: Pointer to the mutex for the.
   * IMU sensor data buffer.
   * @param[in] ang_vel_data_buffer_ptr: Pointer to the buffer of angular
   * velocity sensor data.
   * @param[in] ang_vel_data_buffer_mutex_ptr: Pointer to the mutex for the
   * angular velocity sensor data buffer.
   * @param[in] yaml_filepath: Path to the yaml file for the propagation
   */
  ImuAngVelEKF(IMUQueuePtr imu_data_buffer_ptr,
               std::shared_ptr<std::mutex> imu_data_buffer_mutex_ptr,
               AngularVelocityQueuePtr ang_vel_data_buffer_ptr,
               std::shared_ptr<std::mutex> ang_vel_data_buffer_mutex_ptr,
               const std::string& yaml_filepath);
  /**
   * @brief Constructor for the propagation class
   * @param[in] yaml_filepath: Path to the yaml file for the propagation
   */
  ImuAngVelEKF(const std::string& yaml_filepath);
  /// @}

  /// @name Destructor
  /// @{
  // ======================================================================
  /**
   * @brief Destructor for the imu filter class
   *
   */
  ~ImuAngVelEKF();
  /// @}

  /// @name Filter thread
  /// @{
  // ======================================================================
  /**
   * @brief Start IMU fitler thread
   *
   */
  void StartImuFilterThread();

  // ======================================================================
  /**
   * @brief Run IMU filter. It runs in a new thread with a while loop.
   *
   */
  void RunFilter();

  // ======================================================================
  /**
   * @brief Run the IMU angular velocity EKF filter for one step. This function
   * will also push the latest filtered IMU data to the filtered_imu_data_buffer
   *
   */
  void RunOnce();
  /// @} End of Filter thread

  void LogInputIMU();

  /// @name Getters
  /// @{
  // ======================================================================
  /**
   * @brief Get the filtered imu data buffer
   *
   * @return RobotStateQueuePtr: Pointer to the filtered imu data buffer
   */
  IMUQueuePtr get_filtered_imu_data_buffer_ptr();

  // ======================================================================
  /**
   * @brief Get the filtered_imu data buffer mutex pointer
   *
   * @return std::shared_ptr<std::mutex>: Pointer to the filtered_imu data
   * buffer mutex pointer
   */
  std::shared_ptr<std::mutex> get_filtered_imu_data_buffer_mutex_ptr();

  std::pair<IMUQueuePtr, std::shared_ptr<std::mutex>>
  get_filtered_imu_data_buffer_and_mutex_ptr();
  /// @}

  void InitializeFilter();

  /// @name Initialze IMU bias
  //{
  // ======================================================================
  /**
   * @brief Initialize IMU bias using the static assumption. This assumes
   * the robot is static at a horizontal surface. (i.e. The gravity =
   * /f$9.80 m/s^2/f$ is pointing downward.)
   *
   * The function takes the first n data points, average their value, and
   * subtract the gravity to get the initial bias.
   */
  void InitImuBias();
  ///@} End of Initialize IMU bias

  /// @name Setters
  /// @{
  // ======================================================================
  /**
   * @brief Clear data buffers
   *
   */
  void clear();
  /// @}

  /// @name Adders
  /// @{
  // =======================================================================
  /**
   * @brief Change filter setting to use gyro data from imu meausrement to
   * perform EKF propagation
   *
   * @param[in] imu_data_buffer_ptr: Pointer to the source of gyro / imu
   * measurement
   */
  void add_gyro_propagate(
      IMUQueuePtr imu_data_buffer_ptr,
      std::shared_ptr<std::mutex> imu_data_buffer_mutex_ptr);

  /**
   * @brief add an imu correction
   *
   * @param[in] imu_data_buffer_ptr: Pointer to the buffer of IMU sensor
   * data.
   * @param[in] imu_data_buffer_mutex_ptr: Pointer to the mutex for the.
   * IMU sensor data buffer.
   */
  void add_imu_correction(
      IMUQueuePtr imu_data_buffer_ptr,
      std::shared_ptr<std::mutex> imu_data_buffer_mutex_ptr);

  /**
   * @brief add an angular velocity correction
   *
   * @param[in] ang_vel_data_buffer_ptr: Pointer to the buffer of angular
   * velocity sensor data.
   * @param[in] ang_vel_data_buffer_mutex_ptr: Pointer to the mutex for the
   * angular velocity sensor data buffer.
   */
  void add_ang_vel_correction(
      AngularVelocityQueuePtr ang_vel_data_buffer_ptr,
      std::shared_ptr<std::mutex> ang_vel_data_buffer_mutex_ptr);
  /// @}

  /// @name Macro Correction Methods
  // ======================================================================
  /// @{
  /**
   * @brief Perform correction with one imu measurement
   */
  void SingleImuCorrection();

  /**
   * @brief Perform correction with two or more imu measurements
   */
  void MultiImuCorrection();

  /**
   * @brief Perform correction with one imu measurement and one angular velocity
   * measurement
   */
  void SingleImuAngVelCorrection();

  /**
   * @brief Perform correction with one angular velocity
   */
  void AngVelCorrection();
  /// @}

  /// @name Propagation methods
  // =====================================================================
  /**
   * @brief Perform propagation with imu / gyro measurement
   */
  void GyroPropagate();

  /**
   * @brief Perform propagation with random walk method
   */
  void RandomWalkPropagate();


 private:
  /// @name helper functions
  /// @{
  // ======================================================================
  /**
   * @brief Propagate the angular velocity filter. We assume the state
   * (anguler velocity) undergoes a random walk.
   *
   */
  void AngVelFilterPropagate();
  // ======================================================================
  /**
   * @brief Correct angular velocity with the incoming IMU message.
   *
   * @param[in] imu_measurement: Latest IMU message
   * @return ImuMeasurementPtr: The same IMU measurement with corrected angular
   * velocity
   */
  ImuMeasurementPtr AngVelFilterCorrectIMU(
      const ImuMeasurementPtr& imu_measurement);

  // ======================================================================
  /**
   * @brief Correct angular velocity with the latest wheel encoder measurements.
   * We assume a differential drive model. The angular velocity is obtained from
   * \f$\frac{v_{right_wheel} - v_{left_wheel}}{track_width}\f$
   *
   * @param[in] imu_measurement: Latest imu measurement. We need this to obtain
   * the acceleration information so that we can reconstruct a full imu
   * measurement for the state estimator.
   * @param[in] ang_vel_measurement: Latest angular velocity measurement.
   * @return ImuMeasurementPtr: The same IMU measurement with corrected angular
   * velocity
   */
  ImuMeasurementPtr AngVelFilterCorrectEncoder(
      const ImuMeasurementPtr& imu_measurement,
      const AngularVelocityMeasurementPtr& ang_vel_measurement);

  /// @} // End of helper functions

  IMUQueuePtr
      gyro_prop_data_buffer_ptr_;    // Pointer to gyro / IMU sensor data buffer
  std::shared_ptr<std::mutex> gyro_prop_data_buffer_mutex_ptr_;

  // These are pointer to measurements that will be used in correction step
  IMUQueuePtr imu_data_buffer_ptr_;    // Pointer to the sensor data buffer.
  std::shared_ptr<std::mutex> imu_data_buffer_mutex_ptr_;
  std::vector<IMUQueuePtr>
      imu_data_buffer_ptrs_;    // Vector of imu data buffer pointers
  std::vector<std::shared_ptr<std::mutex>> imu_data_buffer_mutex_ptrs_;

  AngularVelocityQueuePtr ang_vel_data_buffer_ptr_;
  std::shared_ptr<std::mutex> ang_vel_data_buffer_mutex_ptr_;

  double last_propagate_time_
      = -1;    // This will only be modified if user uses GyroPropagte method.
  double t_thres_;    // Threshold between correction time and propagation time

  PROPAGATION_METHOD propagation_method_ = RANDOM_WALK;
  CORRECTION_METHOD correction_method_ = SINGLE_IMU_PLUS_ANG_VEL;

  int num_imu_ = 0;
  int num_ang_vel_ = 0;

  Eigen::Vector3d prev_ang_vel_measurement_ = Eigen::Vector3d::Zero();

  Eigen::Matrix3d
      R_imu2body_inverse_;    // Inverse of rotation matrix that brings
                              // measurement from IMU frame to body frame.
                              // i.e. takes measurements in body frame to
                              // imu frame.
  // IMU bias initialization related variables:
  Eigen::Vector3d bg0_ = Eigen::Vector3d::Zero();    // Gyroscope bias prior.
  Eigen::Vector3d ba0_
      = Eigen::Vector3d::Zero();      // Accelerometer bias prior.
  Eigen::Matrix3d gyro_cov_;          // Gyroscope measurement covariance.
  Eigen::Matrix3d accel_cov_;         // Accelerometer measurement covariance.
  Eigen::Matrix3d gyro_bias_cov_;     // Gyroscope bias covariance.
  Eigen::Matrix3d accel_bias_cov_;    // Accelerometer bias covariance.

  bool enable_imu_bias_update_;    // Boolean value that allows IMU bias update
                                   // during propagation (true for enabling bias
                                   // update, false for disabling).
  bool static_bias_initialization_;    // Flag for static bias initialization
  bool use_imu_ori_est_to_init_;       // Flag for using orientation estimated
                                       // from the imu to perform static bias
                                       // initialization. If set to false, the
  // initial orientation is set to identity.
  // i.e. assumes the robot is on a
  // horizontal flat surface.
  bool bias_initialized_ = false;    // Indicating whether IMU bias has been
                                     // initialized using measurements.
  int init_bias_size_;    // Number of IMU measurements to use for bias
                          // initialization.
  std::vector<Eigen::Matrix<double, 3, 1>,
              Eigen::aligned_allocator<Eigen::Matrix<double, 3, 1>>>
      bias_init_vec_;    // The initialized gyro bias value in the order of
  // [gyro_x, gyro_y, gyro_z].

  bool filter_initialized_ = false;

  // Variables for the angular velocity filter
  ImuMeasurementPtr latest_imu_measurement_ = nullptr;
  Eigen::VectorXd ang_vel_and_bias_est_ = Eigen::VectorXd::Zero(6);
  Eigen::Matrix<double, 6, 6> ang_vel_and_bias_P_;
  Eigen::Matrix<double, 6, 6> ang_vel_and_bias_Q_;    // Process noise
  Eigen::Matrix<double, 3, 3> ang_vel_imu_R_;         // IMU measurement noise
  Eigen::Matrix<double, 3, 3> ang_vel_enc_R_;    // Encoder measurement noise
  Eigen::Matrix<double, 3, 6> H_imu_;
  Eigen::Matrix<double, 3, 6> H_enc_;
  std::vector<Eigen::Matrix<double, 3, 1>,
              Eigen::aligned_allocator<Eigen::Matrix<double, 3, 1>>>
      init_vec_;
  // Thread related:
  std::atomic<bool> stop_thread_ = false;
  std::thread imu_filter_thread_;
  std::shared_ptr<std::mutex> filtered_imu_data_buffer_mutex_ptr_;
  IMUQueuePtr filtered_imu_data_buffer_ptr_;


  // Debugging related:

  std::ofstream imu_propagate_input_outfile_;
  std::ofstream imu_propagate_outfile_;
  std::ofstream imu_ang_vel_outfile_;
  std::ofstream encoder_ang_vel_outfile_;
  std::ofstream filtered_ang_vel_outfile_;

};    // End of class FilteredImuPropagation
}    // namespace imu_filter

#endif    // IMU_FILTER_IMU_ANG_VEL_EKF_H
