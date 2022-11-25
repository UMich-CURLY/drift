/* ----------------------------------------------------------------------------
 * Copyright 2022, Ross Hartley, Tingjun Li
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   base_correction.h
 *  @author Ross Hartley, Tingjun Li
 *  @brief  Header file for Invariant EKF correction method
 *  @date   September 25, 2018
 **/

#ifndef FILTER_INEKF_CORRECTION_BASE_CORRECTION_H
#define FILTER_INEKF_CORRECTION_BASE_CORRECTION_H
#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>
#include "filter/inekf/inekf.h"
#include "filter/noise_params.h"
#include "filter/observations.h"
#include "math/lie_group.h"
#include "state/robot_state.h"
#include "utils/sensor_data_t.h"
#include "utils/utils.h"

namespace inekf {

class Correction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name Constructors
  /**
   * @brief Constructor for the correction class
   *
   * @param[in] sensor_data_buffer: Pointer to the buffer of sensor data
   * @param[in] error_type: Error type for the correction
   */
  Correction(ErrorType error_type);

  /// @name Correction
  /// @{
  // ======================================================================
  /**
   * @brief This is a skeleton for the propagation method. It should be
   * implemented in the child class.
   */
  virtual void Correct(RobotState& state);
  /// @}

 protected:
  ErrorType error_type_;
  const Eigen::Vector3d g_;    // Gravity vector in world frame (z-up)
  Eigen::Vector3d
      magnetic_field_;    // Magnetic field vector in world frame (z-up)
  int aug_map_idx_;       // Index of the augmented map in the aug_maps vector,
                          // which is stored in state, -1 means no map
};                        // class Correction

}    // namespace inekf


#include "../src/filter/inekf/correction/base_correction.cpp"
#endif    // end FILTER_INEKF_CORRECTION_BASE_CORRECTION_H
