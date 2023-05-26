/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   imu_propagation.h
 *  @author Tingjun Li
 *  @brief  Header file for Invariant EKF imu propagation method
 *  Part of the code is modified from Ross Hartley's work:
 *  Paper:
 *  https://journals.sagepub.com/doi/full/10.1177/0278364919894385
 *  Github repo:
 *  https://github.com/RossHartley/invariant-ekf
 *
 *  @date   May 16, 2023
 **/

#ifndef FILTER_INEKF_PROPAGATION_IMU_PROPAGATION_H
#define FILTER_INEKF_PROPAGATION_IMU_PROPAGATION_H

#include <vector>

#include "drift/filter/base_propagation.h"
#include "drift/filter/inekf/inekf.h"
#include "drift/math/lie_group.h"
#include "drift/measurement/imu.h"
#include "drift/utils/type_def.h"

using namespace math;
using namespace state;
using namespace measurement;

namespace filter::inekf {
/**
 * @class ImuPropagation
 * @brief A class for state propagation using imu measurement data.
 *
 * A class for state propagation using imu measurement data. This propagation
 * class holds methods for propagating the state forward by one step using one
 * IMU data. The model is based on the paper:
 * https://journals.sagepub.com/doi/full/10.1177/0278364919894385
 */
class ImuPropagation : public Propagation {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name Constructors
  /// @{
  // ======================================================================
  /**
   * @brief Constructor for the propagation class
   *
   * @param[in] sensor_data_buffer_ptr: Pointer to the buffer of sensor data.
   * @param[in] sensor_data_buffer_mutex_ptr: Pointer to the mutex for the.
   * sensor data buffer.
   * @param[in] error_type: Error type for the propagation. LeftInvariant or
   * RightInvariant
   * @param[in] yaml_filepath: Path to the yaml file for the propagation
   */
  ImuPropagation(IMUQueuePtr sensor_data_buffer_ptr,
                 std::shared_ptr<std::mutex> sensor_data_buffer_mutex_ptr,
                 const ErrorType& error_type, const std::string& yaml_filepath);
  /// @}

  /// @name Propagation
  /// @{
  // ======================================================================
  /**
   * @brief Propagates the estimated state mean and covariance forward using
   * inertial measurements. All landmarks positions are assumed to be static.
   * All contacts' velocities are assumed to be zero + Gaussian noise.
   * The propagation model currently assumes that the covariance is for the
   * right invariant error.
   *
   * @param[in,out] state: state of the robot.
   * @return bool: successfully propagate state or not (if we do not receive a
   * new message and this method is called it'll return false.)
   */
  bool Propagate(RobotState& state) override;
  /// @} End of Propagation

  /// @name Getters
  /// @{
  // ======================================================================
  /**
   * @brief Get the estimate gyro bias.
   *
   * @return const Eigen::Vector3d: Estimated gyro bias. (rad/s)
   */
  const Eigen::Vector3d get_estimate_gyro_bias() const;

  // ======================================================================
  /**
   * @brief Get the estimate accel bias.
   *
   * @return const Eigen::Vector3d:Estimated acceleration bias. (m/s^2)
   */
  const Eigen::Vector3d get_estimate_accel_bias() const;

  // ======================================================================
  /**
   * @brief Check if the bias is initialized or not.
   *
   * @return true: Bias has been successfully initialized or the static bias
   * initialization feature is turned off.
   * @return false: The bias has not been initialized.
   */
  const bool get_bias_initialized() const;

  // ======================================================================
  /**
   * @brief Get the pointer to the sensor data buffer, which is a queue that
   * contains all the measurements received from the sensor.
   *
   * @return const IMUQueuePtr: A smart pointer to the IMU measurement queue.
   * `std::shared_ptr<std::queue<std::shared_ptr<ImuMeasurement<double>>>>`
   */
  const IMUQueuePtr get_sensor_data_buffer_ptr() const;
  /// @} End of Getters

  /// @name Initialze IMU bias
  //{
  // ======================================================================
  /**
   * @brief Initialize IMU bias using the static assumption. This assumes the
   * robot is static at a horizontal surface. (i.e. The gravity = /f$9.80
   * m/s^2/f$ is pointing downward.)
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
   * @brief Set the initial state of the robot according to IMU measurement.
   *
   * @param[in,out] state: The state of the robot, which will be initialized in
   * this method
   * @return bool: whether the initialization is successful
   */
  bool set_initial_state(RobotState& state) override;

  /**
   * @brief Clear the sensor data buffer
   *
   */
  void clear() override;

  /// @} End of Setters

 private:
  /// @name helper functions
  /// @{
  // ======================================================================
  /**
   * @brief computes the discretized state transition matrix in Equation 55 and
   * 58 from Ross Hartley's paper:
   * https://journals.sagepub.com/doi/10.1177/0278364919894385
   *
   * @param[in] w: The unbiased angular velocity measured from an imu (rad/s)
   * @param[in] a: The unbiased linear acceleration measured from an imu (m/s)
   * @param[in] dt: Time step
   * @param[in,out] state: the current state estimate
   *
   * @return Eigen::MatrixXd: the discretized state transition matrix
   */
  Eigen::MatrixXd StateTransitionMatrix(const Eigen::Vector3d& w,
                                        const Eigen::Vector3d& a,
                                        const double dt, RobotState& state);

  // ======================================================================
  /**
   * @brief computes the discretized noise matrix. Details are presented in
   * Equation (52) and (59) to (61) from Ross Hartley's paper:
   * https://journals.sagepub.com/doi/10.1177/0278364919894385
   *
   * @param[in] Phi: The state transition matrix.
   * @param[in] dt: Time step.
   * @param[in,out] state: The robot state.
   *
   * @return Eigen::MatrixXd: The discretized noise matrix
   */
  Eigen::MatrixXd DiscreteNoiseMatrix(const Eigen::MatrixXd& Phi,
                                      const double dt, const RobotState& state);
  /// @} // End of helper functions


  const ErrorType error_type_;            /**< Error type for the propagation.
                                           LeftInvariant or RightInvariant. */
  IMUQueuePtr sensor_data_buffer_ptr_;    // Pointer to the sensor data buffer.
  ImuMeasurementPtr prev_imu_measurement_;    // Previous IMU measurement.

  Eigen::Matrix3d R_imu2body_; /**< Rotation matrix that brings measurement
                                from IMU frame to body frame (meas_body = R
                                * meas_imu). */
  Eigen::Vector3d t_imu2body_; /**< Translation vector that brings measurement
                                from IMU frame to body frame. */

  // IMU bias initialization related variables:
  Eigen::Vector3d bg0_ = Eigen::Vector3d::Zero();    // Gyroscope bias prior.
  Eigen::Vector3d ba0_
      = Eigen::Vector3d::Zero();      // Accelerometer bias prior.
  Eigen::Matrix3d gyro_cov_;          // Gyroscope measurement covariance.
  Eigen::Matrix3d accel_cov_;         // Accelerometer measurement covariance.
  Eigen::Matrix3d gyro_bias_cov_;     // Gyroscope bias covariance.
  Eigen::Matrix3d accel_bias_cov_;    // Accelerometer bias covariance.

  bool enable_imu_bias_update_
      = false; /**< Boolean value that allows IMU bias update
                during propagation (true for enabling bias
                update, false for disabling). */
  bool static_bias_initialization_;    // Flag for static bias initialization
  bool use_imu_ori_to_init_;           /**< Flag for using orientation estimated
                                        from the imu to perform static bias
                                        and robot state initialization. If set
                                        to false, the initial orientation is set to
                                        identity. i.e. assumes the robot is
                                        on a horizontal flat surface. */

  bool bias_initialized_ = false;      /**< Indicating whether IMU bias has been
                                       initialized using measurements. */
  int init_bias_size_; /**< Number of IMU measurements to use for bias
                        initialization. */
  std::vector<Eigen::Matrix<double, 6, 1>,
              Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>>
      bias_init_vec_; /**< The initialized IMU bias value in the order of
                       [gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z]. */

};                    // End of class ImuPropagation
}    // namespace filter::inekf

#endif    // FILTER_INEKF_PROPAGATION_IMU_PROPAGATION_H
