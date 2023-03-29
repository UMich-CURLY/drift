/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   legged_kinematics_correction.h
 *  @author Tingjun Li
 *  @brief  Header file for Invariant EKF kinematic correction method
 *  @date   November 25, 2022
 **/

#ifndef FILTER_INEKF_CORRECTION_KINEMATIC_CORRECTION_H
#define FILTER_INEKF_CORRECTION_KINEMATIC_CORRECTION_H

#include <unordered_map>
#include "filter/base_correction.h"
#include "filter/inekf/inekf.h"
#include "math/lie_group.h"
#include "measurement/legged_kinematics.h"

namespace inekf {
typedef std::shared_ptr<LeggedKinematicsMeasurement> KinematicsMeasurementPtr;
typedef std::queue<std::shared_ptr<LeggedKinematicsMeasurement>>
    KinematicsQueue;
typedef std::shared_ptr<KinematicsQueue> LeggedKinematicsQueuePtr;

struct ContactInfo {
  int id;
  Eigen::Vector3d pose;
  Eigen::Matrix3d cov;
};

/**
 * @class LeggedKinematicsCorrection
 *
 * A class for state correction using foot kinematics and contact events.
 **/
class LeggedKinematicsCorrection : public Correction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name Constructors
  /**
   * @brief Constructor for the correction class
   *
   * @param[in] sensor_data_buffer_ptr: Pointer to the buffer of sensor data
   * @param[in] sensor_data_buffer_mutex_ptr: Pointer to the mutex for the
   * sensor data buffer
   * @param[in] error_type: Error type for the correction. LeftInvariant or
   * RightInvariant
   * @param[in] enable_imu_bias_update: True if the filter should update imu
   * bias
   * @param[in] yaml_filepath: Path of the yaml file for the correction
   * @return bool: successfully correct state or not (if we do not receive a
   * new message and this method is called it'll return false.)
   */
  LeggedKinematicsCorrection(
      LeggedKinematicsQueuePtr sensor_data_buffer_ptr,
      std::shared_ptr<std::mutex> sensor_data_buffer_mutex_ptr,
      const ErrorType& error_type, bool enable_imu_bias_update,
      const std::string& yaml_filepath);

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
   * @param[in,out] state: the current state estimate
   * @return bool: successfully correct state or not (if we do not receive a
   * new message and this method is called it'll return false.)
   */
  bool Correct(RobotState& state);
  /// @}

  /// @name Getters
  /// @{
  // ======================================================================
  /**
   * @brief Return the pointer of the sensor data buffer
   *
   * @return LeggedKinematicsQueuePtr: pointer of the sensor data buffer
   */
  const LeggedKinematicsQueuePtr get_sensor_data_buffer_ptr() const;

  /**
   * @brief Get the initial velocity of the robot
   *
   * @param[in] w: initial angular velocity of the robot
   * @return const Eigen::Vector3d
   */
  const Eigen::Vector3d get_initial_velocity(const Eigen::Vector3d& w) const;
  /// @}

 private:
  const ErrorType error_type_;

  // aug_id_to_column_id map:
  // key: augmented state id
  // value: pointer to the column idx of augmented state's position in the robot
  // state X
  std::unordered_map<int, std::shared_ptr<int>> aug_id_to_column_id_ptr_;

  LeggedKinematicsQueuePtr sensor_data_buffer_ptr_;    // Pointer to the sensor
                                                       // data buffer
  double encoder_std_val_;                // Encoder noise standard deviation
  double kinematics_additive_std_val_;    // Additive noise for kinematics
  Eigen::Matrix3d contact_noise_cov_;     // Contact noise covariance
};                                        // class LeggedKinematicsCorrection
}    // namespace inekf

#endif    // end FILTER_INEKF_CORRECTION_KINEMATIC_CORRECTION_H