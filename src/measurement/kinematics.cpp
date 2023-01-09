/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#include "measurement/kinematics.h"

kinematics::kinematics() : Measurement(KINEMATICS) {
  position_.setZero();
  velocity_.setZero();
}

Eigen::Matrix<double, -1, -1> kinematics::get_J() const { return jacobian_; }
Eigen::Matrix<bool, -1, 1> kinematics::get_contact() const { return contact_; }

Eigen::Matrix<double, -1, 1> kinematics::get_joint_state() const {
  return encoder_position_;
}

Eigen::Matrix<double, 3, 1> kinematics::get_kin_pos() const {
  return position_;
}

Eigen::Matrix<double, 3, 1> kinematics::get_kin_vel() const {
  return velocity_;
}

// Eigen::Matrix<T, 3, 1> kinematics::get_kin_effort() const { return effort_; }
