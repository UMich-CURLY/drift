/* ----------------------------------------------------------------------------
 * Copyright 2022, Ross Hartley, Tingjun Li
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   imu_propagation.h
 *  @author Ross Hartley, Tingjun Li
 *  @brief  Header file for Invariant EKF imu propagation method
 *  @date   September 25, 2018
 **/

#ifndef FILTER_INEKF_PROPAGATION_IMU_PROPAGATION_H
#define FILTER_INEKF_PROPAGATION_IMU_PROPAGATION_H
#include "filter/inekf/propagation/base_propagation.h"

namespace inekf {
template<typename sensor_data_t>
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
   * @param[in] error_type: Error type for the propagation
   */
  ImuPropagation(std::shared_ptr<std::queue<sensor_data_t>> sensor_data_buffer,
                 NoiseParams params, ErrorType error_type);
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
   * @param[in] imu: 6x1 vector containing stacked angular velocity and linear
   * acceleration measurements
   * @param[in] dt: double indicating how long to integrate the inertial
   * measurements for
   * @return None
   */
  void Propagate(RobotState& state, double dt);
  /// @} End of Propagation

 private:
  /// @name helper functions
  /// @{
  // ======================================================================
  /**
   * @brief
   *
   * @param[in] w:
   * @param[in] a:
   * @param[in] dt:
   * @param[in] state:
   *
   * @return Eigen::MatrixXd:
   */
  Eigen::MatrixXd StateTransitionMatrix(Eigen::Vector3d& w, Eigen::Vector3d& a,
                                        double dt, const RobotState& state);

  // ======================================================================
  /**
   * @brief
   *
   * @param[in] phi:
   * @param[in] dt:
   * @param[in] state:
   *
   * @return Eigen::MatrixXd:
   */
  template<int dim = 3>
  Eigen::MatrixXd DiscreteNoiseMatrix(Eigen::MatrixXd& Phi, double dt,
                                      const RobotState& state);
  /// @} // End of helper functions

 private:
  std::shared_ptr<std::queue<sensor_data_t>> sensor_data_buffer_;

};    // End of class ImuPropagation
}    // namespace inekf

#include "../src/filter/inekf/propagation/imu_propagation.cpp"
#endif    // FILTER_INEKF_PROPAGATION_IMU_PROPAGATION_H
