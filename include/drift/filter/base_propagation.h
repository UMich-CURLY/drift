/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   base_propagation.h
 *  @author Tingjun Li
 *  @brief  Header file for base propagation method
 *  @date   May 16, 2023
 **/

#ifndef FILTER_BASE_PROPAGATION_H
#define FILTER_BASE_PROPAGATION_H
#include <Eigen/Dense>
#include <algorithm>
#include <boost/circular_buffer.hpp>
#include <iostream>
#include <map>
#include <mutex>
#include <queue>
#include <vector>
#include "yaml-cpp/yaml.h"

#include "drift/measurement/measurement.h"
#include "drift/state/robot_state.h"
#include "drift/utils/type_def.h"


enum class PropagationType {
  BASE, /**< Base propagation method. */
  IMU,  /**< IMU propagation method. */
};

using namespace measurement;
using namespace state;

namespace filter {
/**
 * @class Propagation
 * @brief Base class for propagation method
 */
class Propagation {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name Constructors
  /// @{
  // ======================================================================
  /**
   * @brief Constructor for the propagation class
   *
   */
  Propagation();

  /**
   * @brief Construct a new Propagation object with mutex
   *
   * @param[in] sensor_data_buffer_mutex_ptr: a pointer to the mutex of the
   * sensor data buffer
   */
  Propagation(std::shared_ptr<std::mutex> sensor_data_buffer_mutex_ptr);
  /// @}

  /// @name Propagation
  /// @{
  // ======================================================================
  /**
   * @brief This is a skeleton for the propagation method. It should be
   * implemented in the child class.
   *
   * @param[in,out] state: The current state of the robot
   * @return bool: successfully propagate state or not (if we do not receive a
   * new message and this method is called it'll return false.)
   */
  virtual bool Propagate(RobotState& state);
  /// @}

  /// @name Getters
  /// @{
  // ======================================================================
  /**
   * @brief Get the pointer of sensor data buffer
   *
   * @return MeasurementQueuePtr: a pointer to the sensor data buffer
   */
  virtual MeasurementQueuePtr get_sensor_data_buffer_ptr();

  /**
   * @brief Get the propagation type
   *
   * @return PropagationType: the propagation type
   */
  const PropagationType get_propagation_type() const;

  /**
   * @brief Get the mutex pointer
   *
   * @return std::shared_ptr<std::mutex>: a pointer to the mutex
   */
  std::shared_ptr<std::mutex> get_mutex_ptr();
  /// @}   // End of getters

  /// @name Setters
  /// @{
  // ======================================================================
  /**
   * @brief Set the initial state of the robot
   *
   * @param[in,out] state: The state of the robot, which will be initialized in
   * this method
   * @return bool: whether the initialization is successful
   */
  virtual bool set_initial_state(RobotState& state);

  /**
   * @brief Clear the sensor data buffer. Need to be override in the child class
   *
   */
  virtual void clear();
  /// @}

 protected:
  const Eigen::Vector3d g_; /**< Gravity vector (m/s^2) in world frame (z-up).
                               Default is 9.80 m/s^2 */
  Eigen::Vector3d
      magnetic_field_; /**< Magnetic field vector in world frame (z-up). */
  PropagationType propagation_type_; /**< The type of the propagation method. */
  std::shared_ptr<std::mutex>
      sensor_data_buffer_mutex_ptr_; /**< The mutex of the sensor data buffer.
                                      */
};                                   // End of class Propagation
}    // namespace filter

#endif    // FILTER_BASE_PROPAGATION_H
