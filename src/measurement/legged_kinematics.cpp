/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#include "measurement/legged_kinematics.h"

LeggedKinematics::LeggedKinematics() : Measurement(KINEMATICS) {}

LeggedKinematics::LeggedKinematics(
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& encoders,
    const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts)
    : encoders_(encoders), contacts_(contacts) {}

void LeggedKinematics::set_contact(
    const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts) {
  contacts_ = contacts;
}
void LeggedKinematics::set_joint_state(
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& encoders) {
  encoders_ = encoders;
}

Eigen::Matrix<double, 3, Eigen::Dynamic> LeggedKinematics::get_J(
    int leg) const {
  int dim = encoders_.rows() / contacts_.rows();
  return jacobian_.block(0, leg * dim, 3, dim);
}

bool LeggedKinematics::get_contact(int leg) const { return contacts_[leg]; }

double LeggedKinematics::get_joint_state(int encoder) const {
  return encoders_[encoder];
}

Eigen::Matrix<double, 3, 1> LeggedKinematics::get_kin_pos(int leg) const {
  return position_.col(leg);
}

// Eigen::Matrix<double, 3, 1> LeggedKinematics::get_kin_vel(int leg) const {
//   return velocity_.col(leg);
// }

// Eigen::Matrix<T, 3, 1> LeggedKinematics::get_kin_effort() const { return
// effort_; }
