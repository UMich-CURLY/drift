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
#include "filter/base_propagation.h"
#include "filter/inekf/inekf.h"
#include "math/lie_group.h"
#include "measurement/imu.h"

namespace inekf {
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
   * @param[in] sensor_data_buffer: Pointer to the buffer of sensor data
   * @param[in] params: Noise parameters for the propagation
   * @param[in] error_type: Error type for the propagation. LeftInvariant or
   * RightInvariant
   */
  ImuPropagation(
      std::shared_ptr<std::queue<ImuMeasurement<double>>> sensor_data_buffer,
      const NoiseParams& params, const ErrorType& error_type,
      const bool estimate_bias = true);
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

  const ErrorType error_type_;
  std::shared_ptr<std::queue<ImuMeasurement<double>>> sensor_data_buffer_;

};    // End of class ImuPropagation
}    // namespace inekf

#endif    // FILTER_INEKF_PROPAGATION_IMU_PROPAGATION_H
