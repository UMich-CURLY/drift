/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   velocity_correction.h
 *  @author Tingjun Li, Tzu-Yuan Lin
 *  @brief  Header file for Invariant EKF velocity correction method
 *  @date   May 16, 2023
 **/

#ifndef FILTER_INEKF_CORRECTION_VELOCITY_CORRECTION_H
#define FILTER_INEKF_CORRECTION_VELOCITY_CORRECTION_H

#include <fstream>
#include <iomanip>
#include <iostream>

#include "drift/filter/base_correction.h"
#include "drift/filter/inekf/inekf.h"
#include "drift/math/lie_group.h"
#include "drift/measurement/velocity.h"
#include "drift/utils/type_def.h"

using namespace math;
using namespace state;
using namespace measurement;

namespace filter::inekf {


/**
 * @class VelocityCorrection
 * @brief A class for state correction using velocity measurement data.
 *
 * A class for state correction using velocity measurement data. This class
 * handles the correction of the state estimate using the measured velocity
 * between the body frame and the world frame. Default is a right-invariant
 * measurement model.
 *
 **/
class VelocityCorrection : public Correction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name Constructors
  /// @{
  /**
   * @brief Constructor for velocity correction class.
   *
   * @param[in] sensor_data_buffer_ptr: Pointer to the buffer of sensor data
   * @param[in] sensor_data_buffer_mutex_ptr: Pointer to the mutex for the
   * sensor data buffer
   * @param[in] error_type: Error type for the correction. LeftInvariant or
   * RightInvariant
   * @param[in] yaml_filepath: Path to the yaml file for the correction
   */
  VelocityCorrection(VelocityQueuePtr sensor_data_buffer_ptr,
                     std::shared_ptr<std::mutex> sensor_data_buffer_mutex_ptr,
                     const ErrorType& error_type,
                     const std::string& yaml_filepath);
  /// @}

  ~VelocityCorrection();

  /// @name Correction Methods
  /// @{
  // ======================================================================
  /**
   * @brief Corrects the state estimate using measured velocity [m/s] that
   * is measured and covarinace of the velocity. Measurements are taken in
   * body frame. This is a right-invariant measurement model.
   *
   * @param[in,out] state: the current state estimate
   * @return bool: successfully correct state or not (if we do not receive a
   * new message and this method is called it'll return false.)
   */
  bool Correct(RobotState& state) override;
  /// @}

  /// @name Getters
  /// @{
  // ======================================================================
  /**
   * @brief Return the pointer of the sensor data buffer
   *
   * @return VelocityQueuePtr: pointer of the sensor data buffer
   */
  const VelocityQueuePtr get_sensor_data_buffer_ptr() const;

  /// @name Setters
  /**
   * @brief Set the initial velocity of the robot
   *
   * @param[in,out] state: the current state estimate, which will be initialized
   * @return bool: whether the initialization is successful
   */
  bool initialize(RobotState& state) override;

  /**
   * @brief Clear the sensor_data_buffer
   *
   */
  void clear() override;
  /// @}


 private:
  const ErrorType error_type_; /**> Error type for the correction,
                               LeftInvariant or RightInvariant. */
  VelocityQueuePtr
      sensor_data_buffer_ptr_; /**> Pointer to the sensor buffer. */
  Eigen::Matrix3d covariance_; /**> Velocity covariance. */
  Eigen::Matrix3d R_vel2body_; /**> Rotation matrix from velocity frame to body
                                  frame. It stores the value from config file
                                  when the class object is created.*/
  double velocity_scale_;

  std::ofstream est_vel_outfile_;
};

}    // namespace filter::inekf


#endif    // end FILTER_INEKF_CORRECTION_VELOCITY_CORRECTION_H