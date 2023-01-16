/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#include "measurement/legged_kinematics.h"

LeggedKinematicsMeasurement::LeggedKinematicsMeasurement()
    : Measurement(LEGGED_KINEMATICS) {}

LeggedKinematicsMeasurement::LeggedKinematicsMeasurement(
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& encoders,
    const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts)
    : encoders_(encoders), contacts_(contacts) {}

void LeggedKinematicsMeasurement::set_contact(
    const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts) {
  contacts_ = contacts;
}
void LeggedKinematicsMeasurement::set_joint_state(
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& encoders) {
  encoders_ = encoders;
}

Eigen::Matrix<double, 3, Eigen::Dynamic> LeggedKinematicsMeasurement::get_J(
    int leg) const {
  int dim = encoders_.rows() / contacts_.rows();
  return jacobian_.block(0, leg * dim, 3, dim);
}

bool LeggedKinematicsMeasurement::get_contact(int leg) const {
  return contacts_[leg];
}

double LeggedKinematicsMeasurement::get_joint_state(int encoder) const {
  return encoders_[encoder];
}

Eigen::Matrix<double, 3, 1> LeggedKinematicsMeasurement::get_kin_pos(
    int leg) const {
  return position_.col(leg);
}

// Eigen::Matrix<double, 3, 1> LeggedKinematicsMeasurement::get_kin_vel(int leg)
// const {
//   return velocity_.col(leg);
// }

// Eigen::Matrix<T, 3, 1> LeggedKinematicsMeasurement::get_kin_effort() const {
// return effort_; }
