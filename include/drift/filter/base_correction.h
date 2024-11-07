/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   base_correction.h
 *  @author Tingjun Li
 *  @brief  Header file for base correction method
 *  @date   May 15, 2023
 **/

#ifndef FILTER_BASE_CORRECTION_H
#define FILTER_BASE_CORRECTION_H

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <vector>

#include <Eigen/Dense>
#include "yaml-cpp/yaml.h"

#include "drift/measurement/measurement.h"
#include "drift/state/robot_state.h"
#include "drift/utils/type_def.h"

enum class CorrectionType { BASE, LEGGED_KINEMATICS, VELOCITY, POSITION };

using namespace measurement;
using namespace state;


namespace filter {
/**
 * @class Correction
 * @brief Base class for correction method. This class just provides a skeleton
 * for the correction method. It should be implemented in the child class.
 */
class Correction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name Constructors
  /// @{
  /**
   * @brief Constructor for the base correction class
   */
  Correction();

  /**
   * @brief Construct a new Correction object with mutex
   *
   * @param[in] sensor_data_buffer_mutex_ptr: a pointer to the mutex of the
   * sensor data buffer
   */
  Correction(std::shared_ptr<std::mutex> sensor_data_buffer_mutex_ptr);
  /// @}

  /// @name Correction
  /// @{
  // ======================================================================
  /**
   * @brief This is a skeleton for the correction method. It should be
   * implemented in the child class.
   *
   * @param[in,out] state: The current state of the robot
   * @return bool: successfully correct state or not (if we do not receive a
   * new message and this method is called it'll return false.)
   */
  virtual bool Correct(RobotState& state);
  /// @}

  /// @name Getters
  /// @{
  // ======================================================================
  /**
   * @brief Get the pointer to the sensor data buffer
   *
   * @return MeasurementQueuePtr: a pointer to the sensor data buffer
   */
  virtual MeasurementQueuePtr get_sensor_data_buffer_ptr();

  /**
   * @brief Get the correction type
   *
   * @return CorrectionType: the correction type
   */
  const CorrectionType get_correction_type() const;

  /**
   * @brief Get the pointer to the mutex of the sensor data buffer
   *
   * @return std::shared_ptr<std::mutex>: a pointer to the mutex of the sensor
   * data buffer
   */
  std::shared_ptr<std::mutex> get_mutex_ptr();
  /// @} End of Getters

  /// @name Setters
  /// @{
  // ======================================================================
  /**
   * @brief Set the initial velocity to the state. This is a dummy method and
   * should be implemented in the child class.
   *
   * @param[in,out] state: the current state estimate, which will be initialized
   * @return bool: whether the initialization is successful
   */
  virtual bool initialize(RobotState& state);

  /**
   * @brief Clear the sensor data buffer. Need to be override in the child class
   *
   */
  virtual void clear();


 protected:
  const Eigen::Vector3d g_; /**< Gravity vector (m/s^2) in world frame
                               (z-up). Default is 9.80 m/s^2 */
  Eigen::Vector3d
      magnetic_field_; /**< Magnetic field vector in world frame (z-up). */
  CorrectionType correction_type_; /**< Correction type. */
  double t_diff_thres_; /**< Threshold for time difference between two
                           measurements. It stores the value from config file
                           when the class object is created*/
  std::shared_ptr<std::mutex>
      sensor_data_buffer_mutex_ptr_; /**< Mutex for the sensor data buffer. */
};    // class Correction

}    // namespace filter

#endif    // end FILTER_BASE_CORRECTION_H
