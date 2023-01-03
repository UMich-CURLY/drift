/* ----------------------------------------------------------------------------
 * Copyright 2022, Tingjun Li, Tzu-Yuan Lin, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   velocity_correction.h
 *  @author Tingjun Li, Tzu-Yuan Lin, Ross Hartley
 *  @brief  Header file for Invariant EKF velocity correction method
 *  @date   November 25, 2022
 **/

#ifndef FILTER_INEKF_CORRECTION_VELOCITY_CORRECTION_H
#define FILTER_INEKF_CORRECTION_VELOCITY_CORRECTION_H
#include "filter/base_correction.h"
#include "filter/inekf/inekf.h"
#include "math/lie_group.h"
#include "measurement/velocity.h"

namespace inekf {
typedef std::queue<std::shared_ptr<VelocityMeasurement<double>>> VelocityQueue;
typedef std::shared_ptr<VelocityQueue> VelocityQueuePtr;

/**
 * @class VelocityCorrection
 *
 * A class for state correction using velocity measurement data.
 **/
class VelocityCorrection : public Correction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name Constructors
  /**
   * @brief Constructor for the correction class
   *
   * @param[in] sensor_data_buffer_ptr: Pointer to the buffer of sensor data
   * @param[in] sensor_data_buffer_mutex_ptr: Pointer to the mutex for the
   * sensor data buffer
   * @param[in] error_type: Error type for the correction. LeftInvariant or
   * RightInvariant
   * @param[in] covariance: Covariance of the velocity measurement
   * @return bool: successfully correct state or not (if we do not receive a
   * new message and this method is called it'll return false.)
   */
  VelocityCorrection(VelocityQueuePtr sensor_data_buffer_ptr,
                     std::shared_ptr<std::mutex> sensor_data_buffer_mutex_ptr,
                     const ErrorType& error_type,
                     const Eigen::Matrix3d& covariance);

  /// @name Correction Methods
  /// @{
  // ======================================================================
  /**
   * @brief Corrects the state estimate using measured velocity [m/s] that is
   * measured and covarinace of the velocity. Measurements are taken in body
   * frameThis is a right-invariant measurement model.
   *
   * @param[in/out] state: the current state estimate
   *
   * @return bool: successfully correct state or not (if we do not receive a
   * new message and this method is called it'll return false.)
   */
  bool Correct(RobotState& state);
  /// @}

  /// @name Getters
  /// @{
  // ======================================================================
  const VelocityQueuePtr get_sensor_data_buffer_ptr() const;

 private:
  const ErrorType error_type_;
  VelocityQueuePtr sensor_data_buffer_ptr_;
  VelocityQueue& sensor_data_buffer_;
  const Eigen::Matrix3d& covariance_;
};

}    // namespace inekf


#endif    // end FILTER_INEKF_CORRECTION_VELOCITY_CORRECTION_H