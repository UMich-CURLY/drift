/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   velocity.h
 *  @author Justin Yu
 *  @brief  Header file for robot velocity estimate
 *  @date   May 16, 2023
 **/

#ifndef MEASUREMENT_VELOCITY_H
#define MEASUREMENT_VELOCITY_H

#include "measurement.h"

namespace measurement {
/**
 * @class VelocityMeasurement
 *
 * @brief measurement class containing robot-frame
 * velocity information.
 */
template<typename T>
class VelocityMeasurement : public Measurement {
 public:
  /// @name Constructors
  /// @{
  /**
   * @brief Default constructor.
   */
  VelocityMeasurement();
  /// @}

  /// @name Setter
  /// @{
  /**
   * @brief Set the velocity measurement coefficients (m/s).
   *
   * @param[in] vx: x velocity coefficient.
   * @param[in] vy: y velocity coefficient.
   * @param[in] vz: z velocity coefficient.
   */
  void set_velocity(T vx, T vy, T vz);
  /// @}

  /// @name Getter
  /// @{
  /**
   * @brief Get the velocity measurement coefficients.
   *
   * @return 3-vector cotaining the robot-frame velocity (m/s).
   */
  Eigen::Matrix<T, 3, 1> get_velocity() const;

  /**
   * @brief Get the velocity vector magnitude.
   *
   * @return magnitude of robot-frame velocity vector (m/s).
   */
  double get_vel_mag() const;

  /**
   * @brief Get the velocity unit vector.
   *
   * @return 3-vector containing normalized robot-frame velocity.
   */
  Eigen::Matrix<T, 3, 1> get_vel_unit_vec() const;
  /// @}

 private:
  Eigen::Matrix<T, 3, 1> vel_;
};
}    // namespace measurement
#include "drift/measurement/impl/velocity_impl.cpp"
#endif    // MEASUREMENT_VELOCITY_H
