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

#include <unordered_map>
#include "filter/base_correction.h"
#include "filter/inekf/inekf.h"
#include "math/lie_group.h"
#include "measurement/kinematics.h"

namespace inekf {

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
   * @param[in] aug_map_type: Type of the augmented states in this correction
   * method. For example, one can use "contact" to indicate the augment state in
   * this correction method are contact positions
   */
  KinematicsCorrection(
      std::shared_ptr<std::queue<KinematicsMeasurement<double>>>
          sensor_data_buffer,
      const ErrorType& error_type, const std::string& aug_type);

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
  // Indicating the type of the augmentation state, e.g. "contact", "landmark"
  const std::string aug_type_;

  // aug_id_to_column_id map:
  // key: augmented state id
  // value: augmented state in the robot state X
  std::unordered_map<int, int> aug_id_to_column_id_;

  std::shared_ptr<std::queue<KinematicsMeasurement<double>>>
      sensor_data_buffer_;
};    // class KinematicsCorrection
}    // namespace inekf

#endif    // end FILTER_INEKF_CORRECTION_KINEMATIC_CORRECTION_H