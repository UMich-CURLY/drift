/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#include "measurement/legged_kinematics.h"

legged_kinematics::legged_kinematics() : Measurement(KINEMATICS) {}

legged_kinematics::legged_kinematics(
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& encoders,
    const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts)
    : encoders_(encoders), contacts_(contacts) {}

void legged_kinematics::set_contact(
    const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts) {
  contacts_ = contacts;
}
void legged_kinematics::set_joint_state(
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& encoders) {
  encoders_ = encoders;
}

Eigen::Matrix<double, 3, Eigen::Dynamic> legged_kinematics::get_J(
    int leg) const {
  int dim = encoders_.rows() / contacts_.rows();
  return jacobian_.block(0, leg * dim, 3, dim);
}

bool legged_kinematics::get_contact(int leg) const { return contacts_[leg]; }

Eigen::Matrix<double, Eigen::Dynamic, 1> legged_kinematics::get_joint_state()
    const {
  return encoders_;
}

Eigen::Matrix<double, 3, 1> legged_kinematics::get_kin_pos(int leg) const {
  return position_.col(leg);
}

Eigen::Matrix<double, 3, 1> legged_kinematics::get_kin_vel(int leg) const {
  return velocity_.col(leg);
}

// Eigen::Matrix<T, 3, 1> legged_kinematics::get_kin_effort() const { return
// effort_; }
