/* ----------------------------------------------------------------------------
 * Copyright 2022, Ross Hartley, Tingjun Li
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   kinematics_correction.h
 *  @author Ross Hartley, Tingjun Li
 *  @brief  Header file for Invariant EKF kinematic correction method
 *  @date   September 25, 2018
 **/

#ifndef FILTER_INEKF_CORRECTION_KINEMATIC_CORRECTION_H
#define FILTER_INEKF_CORRECTION_KINEMATIC_CORRECTION_H
#include "filter/inekf/correction/base_correction.h"

namespace inekf {

template<typename sensor_data_t>
class KinematicsCorrection : public Correction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name Constructors
  /**
   * @brief Constructor for the correction class
   *
   * @param[in] sensor_data_buffer: Pointer to the buffer of sensor data
   * @param[in] error_type: Error type for the correction
   * @param[in] aug_map_idx: Index of the augmented map in state_'s aug_map
   * vector
   */
  KinematicsCorrection(
      std::shared_ptr<std::queue<sensor_data_t>> sensor_data_buffer,
      ErrorType error_type, int aug_map_idx);

  /// @name Correction Methods
  /// @{
  // ======================================================================
  /**
   * @brief Corrects the state estimate using the measured forward kinematics
   * between the IMU and a set of contact frames. If contact is indicated but
   * not included in the state, the state is augmented to include the estimated
   * contact position. If contact is not indicated but is included in the state,
   * the contact position is marginalized out of the state. This is a
   * right-invariant measurement model. Example usage can be found in @include
   * kinematics.cpp
   * @param[in] measured_kinematics: the measured kinematics containing the
   * contact id, relative pose measurement in the IMU frame, and covariance
   * @return None
   */
  void Correct(RobotState& state);
  /// @}

 private:
  std::shared_ptr<std::queue<sensor_data_t>> sensor_data_buffer_;
  int aug_map_idx_;    // Index of the augmented map in the aug_maps vector,
                       // which is stored in state
};                     // class KinematicsCorrection
}    // namespace inekf

#include "../src/filter/inekf/correction/kinematics_correction.cpp"
#endif    // end FILTER_INEKF_CORRECTION_KINEMATIC_CORRECTION_H