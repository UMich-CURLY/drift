/* ----------------------------------------------------------------------------
 * Copyright 2022, Tingjun Li, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   imu_propagation.h
 *  @author Tingjun Li, Ross Hartley
 *  @brief  Header file for Invariant EKF imu propagation method
 *  @date   November 25, 2022
 **/

#ifndef FILTER_INEKF_PROPAGATION_IMU_PROPAGATION_H
#define FILTER_INEKF_PROPAGATION_IMU_PROPAGATION_H
#include <vector>
#include "filter/base_propagation.h"
#include "filter/inekf/inekf.h"
#include "math/lie_group.h"
#include "measurement/imu.h"

namespace inekf {
typedef std::queue<std::shared_ptr<ImuMeasurement<double>>> IMUQueue;
typedef std::shared_ptr<IMUQueue> IMUQueuePtr;

/**
 * @class ImuPropagation
 *
 * A class for state propagation using imu measurement data.
 **/
class ImuPropagation : public Propagation {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name Constructors
  /// @{
  // ======================================================================
  /**
   * @brief Constructor for the propagation class
   *
   * @param[in] sensor_data_buffer_ptr: Pointer to the buffer of sensor data
   * @param[in] sensor_data_buffer_mutex_ptr: Pointer to the mutex for the
   * sensor data buffer
   * @param[in] params: Noise parameters for the propagation
   * @param[in] error_type: Error type for the propagation. LeftInvariant or
   * @param[in] estimate_bias: Whether to estimate the bias
   * @param[in] imu2body: The transformation from imu frame to body frame
   * @param[in] static_bias_initialization: Whether to initialize the bias
   * RightInvariant
   */
  ImuPropagation(IMUQueuePtr sensor_data_buffer_ptr,
                 std::shared_ptr<std::mutex> sensor_data_buffer_mutex_ptr,
                 const NoiseParams& params, const ErrorType& error_type,
                 const bool estimate_bias = true,
                 const std::vector<double>& imu2body = {1, 0, 0, 0},
                 const bool static_bias_initialization = true);
  /// @}

  /// @name Propagation
  /// @{
  // ======================================================================
  /**
   * @brief Propagates the estimated state mean and covariance forward using
   * inertial measurements. All landmarks positions are assumed to be static.
   * All contacts velocities are assumed to be zero + Gaussian noise.
   * The propagation model currently assumes that the covariance is for the
   * right invariant error.
   *
   * @param[in/out] state: state of the robot
   * @return None
   */
  void Propagate(RobotState& state);
  /// @} End of Propagation

  /// @name Getters
  /// @{
  // ======================================================================
  const Eigen::Vector3d get_estimate_gyro_bias() const;

  // ======================================================================
  const Eigen::Vector3d get_estimate_accel_bias() const;

  // ======================================================================
  const bool get_bias_initialized() const;

  // ======================================================================
  const IMUQueuePtr get_sensor_data_buffer_ptr() const;
  /// @} End of Getters

  // ======================================================================
  void InitImuBias();


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
   * @param[in/out] state: the current state estimate
   *
   * @return Eigen::MatrixXd: the discretized state transition matrix
   */
  Eigen::MatrixXd StateTransitionMatrix(const Eigen::Vector3d& w,
                                        const Eigen::Vector3d& a,
                                        const double dt,
                                        const RobotState& state);

  // ======================================================================
  /**
   * @brief computes the discretized noise matrix. Details are presented in
   * Equation (52) and (59) to (61) from Ross Hartley's paper:
   * https://journals.sagepub.com/doi/10.1177/0278364919894385
   *
   * @param[in] Phi: The state transition matrix.
   * @param[in] dt: Time step
   * @param[in/out] state: The robot state
   *
   * @return Eigen::MatrixXd: The discretized noise matrix
   */
  Eigen::MatrixXd DiscreteNoiseMatrix(const Eigen::MatrixXd& Phi,
                                      const double dt, const RobotState& state);
  /// @} // End of helper functions

  static Eigen::Matrix3d compute_R_imu2body(const std::vector<double> imu2body);


  const ErrorType error_type_;
  IMUQueuePtr sensor_data_buffer_ptr_;
  IMUQueue& sensor_data_buffer_;
  std::shared_ptr<std::mutex> sensor_data_buffer_mutex_ptr_;
  const Eigen::Matrix3d R_imu2body_;

  // IMU bias initialization related variables:
  Eigen::Vector3d bg0_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba0_ = Eigen::Vector3d::Zero();
  const bool estimate_bias_;
  bool static_bias_initialization_ = false;
  bool estimator_debug_enabled_ = false;
  bool use_imu_ori_est_init_bias_ = false;
  bool bias_initialized_ = false;
  int init_bias_size_;    // Number of IMU measurements to use for bias
                          // initialization
  std::vector<Eigen::Matrix<double, 6, 1>,
              Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>>
      bias_init_vec_;

};    // End of class ImuPropagation
}    // namespace inekf

#endif    // FILTER_INEKF_PROPAGATION_IMU_PROPAGATION_H
