/* ----------------------------------------------------------------------------
 * Copyright 2022, Tingjun Li, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   base_correction.h
 *  @author Tingjun Li, Ross Hartley
 *  @brief  Header file for Invariant EKF base correction method
 *  @date   November 25, 2022
 **/

#ifndef FILTER_BASE_CORRECTION_H
#define FILTER_BASE_CORRECTION_H
#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <queue>
#include <vector>
#include "filter/noise_params.h"
#include "filter/observations.h"
#include "state/robot_state.h"

enum class CorrectionType {
  BASE,
  KINEMATICS,
  VELOCITY,
};
class Correction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name Constructors
  /**
   * @brief Constructor for the base correction class
   */
  Correction();

  /// @name Correction
  /// @{
  // ======================================================================
  /**
   * @brief This is a skeleton for the correction method. It should be
   * implemented in the child class.
   *
   * @param[in/out] state: The current state of the robot
   */
  virtual void Correct(RobotState& state);
  /// @}

  CorrectionType get_correction_type();

 protected:
  const Eigen::Vector3d g_;    // Gravity vector in world frame (z-up)
  Eigen::Vector3d
      magnetic_field_;    // Magnetic field vector in world frame (z-up)
  CorrectionType correction_type_;
};    // class Correction

// #include "../src/filter/base_correction.cpp"
#endif    // end FILTER_BASE_CORRECTION_H
