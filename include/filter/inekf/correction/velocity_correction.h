/* ----------------------------------------------------------------------------
 * Copyright 2022, Ross Hartley, Tingjun Li
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   base_correction.h
 *  @author Ross Hartley, Tingjun Li
 *  @brief  Header file for Invariant EKF correction methods
 *  @date   September 25, 2018
 **/

#ifndef FILTER_INEKF_CORRECTION_VELOCITY_CORRECTION_H
#define FILTER_INEKF_CORRECTION_VELOCITY_CORRECTION_H
#include "filter/inekf/correction/base_correction.h"

namespace inekf {

template<typename sensor_data_t>
class VelocityCorrection : public Correction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name Constructors
  /**
   * @brief Constructor for the correction class
   *
   * @param[in] sensor_data_buffer: Pointer to the buffer of sensor data
   * @param[in] error_type: Error type for the correction
   * @param[in] covariance: Covariance of the velocity measurement
   */
  VelocityCorrection(
      std::shared_ptr<std::queue<sensor_data_t>> sensor_data_buffer,
      ErrorType error_type, const Eigen::Matrix3d& covariance);

  /// @name Correction Methods
  /// @{
  // ======================================================================
  /**
   * @brief Corrects the state estimate using measured velocity and covarinace
   * the velocity. This is a right-invariant measurement model.
   * @param[in] measured_velocity: the measured velocity
   * @param[in] measured_velocity_covariance: the measured velocity covariance
   * @param[in] state: the current state estimate
   *
   * @return None
   */
  void Correct(RobotState& state);
  /// @}
 private:
  std::shared_ptr<std::queue<sensor_data_t>> sensor_data_buffer_;
  const Eigen::Matrix3d covariance_;
};

}    // namespace inekf

#include "../src/filter/inekf/correction/velocity_correction.cpp"
#endif    // end FILTER_INEKF_CORRECTION_VELOCITY_CORRECTION_H