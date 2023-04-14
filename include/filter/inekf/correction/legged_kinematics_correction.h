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

using namespace math;
using namespace state;
using namespace measurement;

namespace filter::inekf {
typedef std::shared_ptr<LeggedKinematicsMeasurement>
    KinematicsMeasurementPtr; /**< Type: Shared pointer to a
                                 LeggedKinematicsMeasurement object. */
typedef std::queue<std::shared_ptr<LeggedKinematicsMeasurement>>
    KinematicsQueue; /**< Type: Queue of LeggedKinematicsMeasurementPtr objects.
                      */
typedef std::shared_ptr<KinematicsQueue>
    LeggedKinematicsQueuePtr; /**< Type: Shared pointer
                                 to a KinematicsQueue object. */

/**
 * @brief A struct for contact information
 *
 * @param[in] id: Contact foot(leg)'s id
 * @param[in] pose: Contact foot(leg)'s pose
 * @param[in] cov: Contact foot(leg)'s pose covariance
 */
struct ContactInfo {
  int id;               /**< ID of the contact foot(leg). */
  Eigen::Vector3d pose; /**< Pose of the contact foot(leg). */
  Eigen::Matrix3d cov;  /**< Covariance of the contact foot(leg). */
};

/**
 * @class LeggedKinematicsCorrection
 *
 * @brief A class for state correction using foot kinematics and contact events.
 *
 * A class for state correction using foot kinematics and contact events. This
 * class handles the correction of the state estimate using the measured forward
 * kinematics between the body frame and a set of contact frames. If contact is
 * indicated but not included in the state, the state is augmented to include
 * include the estimated contact foot position. If contact is not indicated but
 * is included in the robot state, the contact position is marginalized out of
 * the state. Default is a right-invariant measurement model.
 **/
class LeggedKinematicsCorrection : public Correction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name Constructors
  /**
   * @brief Constructor for the legged kinematics correction class.
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
   * the contact position is marginalized out of the state. Default is a
   * right-invariant measurement model.
   *
   * @param[in,out] state: the current state estimate
   * @return bool: successfully correct state or not (if we do not receive a
   * new message and this method is called it'll return false.)
   */
  bool Correct(RobotState& state) override;
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

  /// @name Setters
  /// @{
  // ======================================================================
  /**
   * @brief Set the initial velocity of the robot
   *
   * @param[in,out] state: the current state estimate, which will be initialized
   */
  void set_initial_velocity(RobotState& state) override;
  /// @}

 private:
  const ErrorType error_type_; /**> Error type for the correction. LeftInvariant
                                  or RightInvariant */

  /**>
   * @var map:
   * key: augmented state id
   * value: pointer to the column idx of augmented state's position in the robot
   * state X
   */
  std::unordered_map<int, std::shared_ptr<int>> aug_id_to_column_id_ptr_;

  LeggedKinematicsQueuePtr sensor_data_buffer_ptr_; /**> Pointer to the sensor
                                                     data buffer. */
  double encoder_std_val_; /**> Encoder noise standard deviation value. It
                              stores the value from config file when the class
                              object is created. */
  double kinematics_additive_std_val_; /**> Additive noise standard deviation
                                          valuefor kinematics. It stores the
                                          value from config file when the class
                                          object is created. */
  Eigen::Matrix3d contact_noise_cov_;  /**> Contact noise covariance. */
};                                     // class LeggedKinematicsCorrection
}    // namespace filter::inekf

#endif    // end FILTER_INEKF_CORRECTION_KINEMATIC_CORRECTION_H