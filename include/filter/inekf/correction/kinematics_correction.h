/* ----------------------------------------------------------------------------
 * Copyright 2022, Tingjun Li, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   kinematics_correction.h
 *  @author Tingjun Li, Ross Hartley
 *  @brief  Header file for Invariant EKF kinematic correction method
 *  @date   November 25, 2022
 **/

#ifndef FILTER_INEKF_CORRECTION_KINEMATIC_CORRECTION_H
#define FILTER_INEKF_CORRECTION_KINEMATIC_CORRECTION_H
#include "filter/base_correction.h"
#include "filter/inekf/inekf.h"
#include "math/lie_group.h"

namespace inekf {

template<typename sensor_data_t>
/**
 * @class KinematicsCorrection
 *
 * A class for state correction using foot kinematics and contact events.
 **/
class KinematicsCorrection : public Correction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name Constructors
  /**
   * @brief Constructor for the correction class
   *
   * @param[in] sensor_data_buffer: Pointer to the buffer of sensor data
   * @param[in] error_type: Error type for the correction. LeftInvariant or
   * RightInvariant
   * @param[in] aug_map_idx: Index of the augmented map in state_'s aug_map. For
   * example, if the state_ has 2 augmented maps in its augmented maps
   * container, Without lost of generality, let's assume the first one is
   * contact map, and the second one is landmark map. In this case, for
   * kinematics correction, aug_map_idx = 1. This input will be passed to the
   * constructor from the StateEstimator class automatically and users don't
   * need to pass it by theirselves.
   */
  KinematicsCorrection(
      std::shared_ptr<std::queue<sensor_data_t>> sensor_data_buffer,
      const ErrorType& error_type, int aug_map_idx);

  /// @name Correction Methods
  /// @{
  // ======================================================================
  /**
   * @brief Corrects the state estimate using the measured forward kinematics
   * between the IMU and a set of contact frames. If contact is indicated but
   * not included in the state, the state is augmented to include the estimated
   * contact position. If contact is not indicated but is included in the state,
   * the contact position is marginalized out of the state. This is a
   * right-invariant measurement model.
   *
   * @param[in/out] state: the current state estimate
   * @return None
   */
  void Correct(RobotState& state);
  /// @}

 private:
  const ErrorType error_type_;
  std::shared_ptr<std::queue<sensor_data_t>> sensor_data_buffer_;
  int aug_map_idx_;    // Index of the augmented map in the aug_maps vector,
                       // which is stored in state
};                     // class KinematicsCorrection
}    // namespace inekf

#include "../src/filter/inekf/correction/kinematics_correction.cpp"
#endif    // end FILTER_INEKF_CORRECTION_KINEMATIC_CORRECTION_H