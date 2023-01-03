/* ----------------------------------------------------------------------------
 * Copyright 2022, Tingjun Li, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   base_propagation.h
 *  @author Tingjun Li, Ross Hartley
 *  @brief  Header file for Invariant EKF base propagation method
 *  @date   November 25, 2022
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
#include "filter/noise_params.h"
#include "measurement/measurement.h"
#include "state/robot_state.h"

enum class PropagationType {
  BASE,
  IMU,
};

typedef std::queue<std::shared_ptr<Measurement>> MeasurementQueue;
typedef std::shared_ptr<MeasurementQueue> MeasurementQueuePtr;

class Propagation {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name Constructors
  /// @{
  // ======================================================================
  /**
   * @brief Constructor for the propagation class
   *
   * @param[in] params: The noise parameter for propagation
   */
  /// @}
  Propagation(const NoiseParams& params);

  /// @name Propagation
  /// @{
  // ======================================================================
  /**
   * @brief This is a skeleton for the propagation method. It should be
   * implemented in the child class.
   *
   * @param[in/out] state: The current state of the robot
   * @return bool: successfully propagate state or not (if we do not receive a
   * new message and this method is called it'll return false.)
   */
  virtual bool Propagate(RobotState& state);
  /// @}

  /// @name Getters
  /// @{
  // ======================================================================
  /**
   * @brief Gets the current noise parameters.
   *
   * @return inekf::NoiseParams: The current noise parameters.
   */
  const NoiseParams get_noise_params() const;
  /// @}

  /// @name Getters
  virtual MeasurementQueuePtr get_sensor_data_buffer_ptr();

  const PropagationType get_propagation_type() const;

 protected:
  const NoiseParams noise_params_;
  const Eigen::Vector3d g_;    // Gravity vector in world frame (z-up)
  Eigen::Vector3d
      magnetic_field_;    // Magnetic field vector in world frame (z-up)
  PropagationType propagation_type_;
};    // End of class Propagation


// #include "../src/filter/base_propagation.cpp"
#endif    // FILTER_BASE_PROPAGATION_H
