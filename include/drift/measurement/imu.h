/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   imu.h
 *  @author Justin Yu
 *  @brief  Header file for robot imu estimate measurement
 *  @date   May 16, 2023
 **/

#ifndef MEASUREMENT_IMU_H
#define MEASUREMENT_IMU_H

#include "measurement.h"

namespace measurement {
/**
 * @class ImuMeasurement
 *
 * @brief measurement class containing intertial
 * measurement unit estimated parameters.
 */
template<typename T>
class ImuMeasurement : public Measurement {
 public:
  /// @name Constructors
  /// @{
  /**
   * @brief Default constructor.
   */
  ImuMeasurement();
  /// @}

  /// @name Setter
  /// @{
  /**
   * @brief Set the imu measurement quaternion coefficients.
   * @warning Arguments must be normalized.
   * @param[in] w: real quaternion coefficient.
   * @param[in] x: imaginary quaternion coefficient.
   * @param[in] y: imaginary quaternion coefficient.
   * @param[in] z: imaginary quaternion coefficient.
   */
  void set_quaternion(T w, T x, T y, T z);

  /**
   * @brief Set the imu measurement angular velocity (rad/s).
   *
   * @param[in] x: axis coefficient.
   * @param[in] y: axis coefficient.
   * @param[in] z: axis coefficient.
   */
  void set_angular_velocity(T x, T y, T z);

  /**
   * @brief Set the imu measurement linear acceleration (m/s^2).
   *
   * @param[in] x: axis coefficient.
   * @param[in] y: axis coefficient.
   * @param[in] z: axis coefficient.
   */
  void set_lin_acc(T x, T y, T z);
  /// @}

  /// @name Getter
  /// @{
  /**
   * @brief Get the orthonormal 3D rotation matrix for the imu
   * measurement.
   *
   * @return 3x3 Matrix in SO(3).
   */
  Eigen::Matrix<T, 3, 3> get_rotation_matrix() const;

  /**
   * @brief Get the imu measurement quaternion coefficients.
   *
   * @return The quaternion (w, x, y, z).
   */
  Eigen::Quaternion<T> get_quaternion() const;

  /**
   * @brief Get the imu measurement angular velocity coefficients (rad/s).
   *
   * @return The angular velocity (x, y, z).
   */
  Eigen::Matrix<T, 3, 1> get_angular_velocity() const;

  /**
   * @brief Get the imu measurement linear acceleration coefficients (m/s^2).
   *
   * @return The linear acceleration (x, y, z).
   */
  Eigen::Matrix<T, 3, 1> get_lin_acc() const;
  /// @}

 private:
  Eigen::Quaternion<T> quaternion_;
  Eigen::Matrix<T, 3, 1> angular_velocity_;
  Eigen::Matrix<T, 3, 1> linear_acceleration_;

  /**
   * @brief Validate quaternion inputs.
   *
   * Quaternion must be normalized for well
   * defined Eigen::toRotationMatrix() output
   */
  void validate_quat(T w, T x, T y, T z);
};
}    // namespace measurement
#include "drift/measurement/impl/imu_impl.cpp"

#endif    // MEASUREMENT_IMU_H
