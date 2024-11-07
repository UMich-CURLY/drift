/* ----------------------------------------------------------------------------
 * Copyright 2024, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   position_correction.h
 *  @author Surya Singh
 *  @brief  Header file for Invariant EKF position correction method
 *  @date   Oct 11, 2024
 **/

#ifndef FILTER_INEKF_CORRECTION_POSITION_CORRECTION_H
#define FILTER_INEKF_CORRECTION_POSITION_CORRECTION_H

#include <fstream>
#include <iomanip>
#include <iostream>

#include "drift/filter/base_correction.h"
#include "drift/filter/inekf/inekf.h"
#include "drift/math/lie_group.h"
#include "drift/measurement/odom.h"
#include "drift/utils/type_def.h"

using namespace math;
using namespace state;
using namespace measurement;

namespace filter::inekf {


/**
 * @class PositionCorrection
 * @brief A class for state correction using position measurement data.
 *
 * A class for state correction using position measurement data. This class
 * handles the correction of the state estimate using the measured position
 * between the body frame and the world frame. Default is a left-invariant
 * measurement model.
 *
 **/
class PositionCorrection : public Correction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name Constructors
  /// @{
  /**
   * @brief Constructor for position correction class.
   *
   * @param[in] sensor_data_buffer_ptr: Pointer to the buffer of sensor data
   * @param[in] sensor_data_buffer_mutex_ptr: Pointer to the mutex for the
   * sensor data buffer
   * @param[in] error_type: Error type for the correction. LeftInvariant or
   * RightInvariant
   * @param[in] yaml_filepath: Path to the yaml file for the correction
   */
  PositionCorrection(OdomQueuePtr sensor_data_buffer_ptr,
                     std::shared_ptr<std::mutex> sensor_data_buffer_mutex_ptr,
                     const ErrorType& error_type,
                     const std::string& yaml_filepath);
  /// @}

  ~PositionCorrection();

  /// @name Correction Methods
  /// @{
  // ======================================================================
  /**
   * @brief Corrects the state estimate using measured position that
   * is measured and covarinace of the position. Measurements are taken in
   * world frame. This is a left-invariant measurement model.
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
   * @return OdomQueuePtr: pointer of the sensor data buffer
   */
  const OdomQueuePtr get_sensor_data_buffer_ptr() const;

  /// @name Setters
  /**
   * @brief Set the initial position of the robot
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
  const ErrorType error_type_;             /**> Error type for the correction,
                                           LeftInvariant or RightInvariant. */
  OdomQueuePtr sensor_data_buffer_ptr_;    /**> Pointer to the sensor buffer. */
  Eigen::Matrix<double, 3, 3> covariance_; /**> Position covariance. */

  double t_diff_thres_;

  std::ofstream est_pose_outfile_;
};

}    // namespace filter::inekf


#endif    // end FILTER_INEKF_CORRECTION_POSITION_CORRECTION_H