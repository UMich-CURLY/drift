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

  virtual void compute_kinematics() = 0;

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> get_J() const;

  Eigen::Matrix<bool, Eigen::Dynamic, 1> get_contact() const;

  Eigen::Matrix<double, Eigen::Dynamic, 1> get_joint_state() const;

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
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> jacobian_;
  Eigen::Matrix<bool, Eigen::Dynamic, 1> contact_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> encoder_position_;
  Eigen::Matrix<double, 3, Eigen::Dynamic> body_to_foot_;
};


#endif