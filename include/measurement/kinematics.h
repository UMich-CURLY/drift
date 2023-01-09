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
 * @class kinematics
 *
 * Derived measurement class containing information
 * about world-frame robot state
 */
class kinematics : public Measurement {
 public:
  /**
   * @brief Default constructor.
   */
  kinematics();

  /**
   * @brief Computes
   *
   * @param[in] ct Vector of contact states
   */
  virtual void compute_kinematics() = 0;

  /**
   * @brief Set the ground contact state vector.
   *
   * @param[in] ct Vector of contact states
   */
  void set_contact(const Eigen::Matrix<bool, -1, 1>& ct);

  /**
   * @brief Set the encoder position state vector.
   *
   * @param[in] js Vector of encoder position coefficients (rad)
   */
  void set_joint_state(const Eigen::Matrix<double, -1, 1>& js);

  /**
   * @brief Get the Jacobian matrix.
   *
   * @return Jacobian Matrix
   */
  Eigen::Matrix<double, -1, -1> get_J() const;

  /**
   * @brief Get the ground contact state vector.
   *
   * @return Vector of contact states
   */
  Eigen::Matrix<bool, -1, 1> get_contact() const;

  /**
   * @brief Get the encoder position state vector.
   *
   * @return Vector of encoder position coefficients (rad)
   */
  Eigen::Matrix<double, -1, 1> get_joint_state() const;

  /**
   * @brief Get the world-frame position in Euclidean space.
   *
   * @return 3-vector of Kinematics position coefficients (m)
   */
  Eigen::Matrix<double, 3, 1> get_kin_pos() const;

  /**
   * @brief Get the world-frame velocity in Euclidean space.
   *
   * @return 3-vector of Kinematics velocity (m/s)
   */
  Eigen::Matrix<double, 3, 1> get_kin_vel() const;

  /**
   * @brief Get the world-frame effort (force) in Euclidean space.
   *
   * @return 3-vector of Kinematics effort (Newtons).
   */
  // Eigen::Matrix<double, 3, 1> get_kin_effort() const;

 protected:
  Eigen::Matrix<double, 3, 1> position_;
  Eigen::Matrix<double, 3, 1> velocity_;
  // Eigen::Matrix<T, 3, 1> effort_;
  Eigen::Matrix<double, -1, -1> jacobian_;
  Eigen::Matrix<bool, -1, 1> contact_;
  Eigen::Matrix<double, -1, 1> encoder_position_;
};


#endif