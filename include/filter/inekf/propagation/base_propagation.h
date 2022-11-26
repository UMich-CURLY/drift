/* ----------------------------------------------------------------------------
 * Copyright 2022, Ross Hartley, Tingjun Li
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   base_propagation.h
 *  @author Ross Hartley, Tingjun Li
 *  @brief  Header file for Invariant EKF base propagation method
 *  @date   September 25, 2018
 **/

#ifndef FILTER_INEKF_PROPAGATION_BASE_PROPAGATION_H
#define FILTER_INEKF_PROPAGATION_BASE_PROPAGATION_H
#include <Eigen/Dense>
#include <algorithm>
#include <boost/circular_buffer.hpp>
#include <iostream>
#include <map>
#include <queue>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>
#include "filter/inekf/inekf.h"
#include "filter/noise_params.h"
#include "filter/observations.h"
#include "math/lie_group.h"
#include "state/robot_state.h"
#include "utils/sensor_data_t.h"

namespace inekf {

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
   * @param[in] error_type: Error type for the propagation
   */
  /// @}
  Propagation(NoiseParams params, ErrorType error_type);

  /// @name Propagation
  /// @{
  // ======================================================================
  /**
   * @brief This is a skeleton for the propagation method. It should be
   * implemented in the child class.
   *
   * @param[in] state: The current state of the robot
   * @param[in] dt: The time step for the propagation
   */
  virtual void Propagate(RobotState& state, double dt);
  /// @}

  /// @name Setters
  /// @{
  // ======================================================================
  /**
   * @brief Sets the current noise parameters
   *
   * @param[in] params: The noise parameters to be assigned.
   * @return None
   */
  void set_noise_params(NoiseParams params);


  /// @name Getters
  /// @{
  // ======================================================================
  /**
   * @brief Gets the current noise parameters.
   *
   * @param[in] None
   * @return inekf::NoiseParams: The current noise parameters.
   */
  NoiseParams get_noise_params() const;
  /// @}

 protected:
  NoiseParams noise_params_;
  ErrorType error_type_;
  const Eigen::Vector3d g_;    // Gravity vector in world frame (z-up)
  Eigen::Vector3d
      magnetic_field_;    // Magnetic field vector in world frame (z-up)
  bool estimate_bias_
      = true;    // Whether to estimate the gyro and accelerometer biases
};               // End of class Propagation

}    // namespace inekf

#include "../src/filter/inekf/propagation/base_propagation.cpp"
#endif    // FILTER_INEKF_PROPAGATION_BASE_PROPAGATION_H
