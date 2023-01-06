/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   kinematics.h
 *  @author Justin Yu
 *  @brief  Header file for robot kinematics state measurement
 *  @date   Nov 16, 2022
 **/

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "measurement.h"

/**
 * @class KinematicsMeasurement
 *
 * Derived measurement class containing information
 * about world-frame robot state
 */
template<typename T>
class KinematicsMeasurement : public Measurement {
 public:
  /**
   * @brief Default constructor.
   */
  KinematicsMeasurement();

  /**
   * @brief Set the world-frame kinematics state coefficients.
   *
   * @param[in] position: vector of kinematics position coefficients (m).
   * @param[in] velocity: vector of kinematics velocity coefficients (m/s).
   * @param[in] effort: vector of kinematics effort coefficients (Newtons).
   */
  void set_kin_state(const Eigen::Matrix<T, 3, 1>& position,
                     const Eigen::Matrix<T, 3, 1>& velocity,
                     const Eigen::Matrix<T, 3, 1>& effort);

  /**
   * @brief Get the world-frame position in Euclidean space.
   *
   * @return 3-vector of kinematics position coefficients (m)
   */
  Eigen::Matrix<T, 3, 1> get_kin_pos() const;

  /**
   * @brief Get the world-frame velocity in Euclidean space.
   *
   * @return 3-vector of kinematics velocity (m/s)
   */
  Eigen::Matrix<T, 3, 1> get_kin_vel() const;

  /**
   * @brief Get the world-frame effort (force) in Euclidean space.
   *
   * @return 3-vector of kinematics effort (Newtons).
   */
  Eigen::Matrix<T, 3, 1> get_kin_effort() const;

 private:
  Eigen::Matrix<T, 3, 1> position_;
  Eigen::Matrix<T, 3, 1> velocity_;
  Eigen::Matrix<T, 3, 1> effort_;
};
#include "measurement/impl/kinematics_impl.cpp"

#endif