/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   tensegrity_kinematics.cpp
 *  @author Wenzhe Tong
 *  @brief  Source code for tensegrity robot kinematics state measurement
 *  @date   Feb 22, 2023
 **/

#include "drift/measurement/tensegrity_kinematics.h"

namespace measurement {
TensegrityKinematicsMeasurement::TensegrityKinematicsMeasurement()
    : Measurement(TENSEGRITY_KINEMATICS) {}

TensegrityKinematicsMeasurement::TensegrityKinematicsMeasurement(
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& encoders,
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& d_encoders,
    const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts)
    : encoders_(encoders), d_encoders_(d_encoders), contacts_(contacts) {}

void TensegrityKinematicsMeasurement::set_contact(
    const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts) {
  contacts_ = contacts;
}

void TensegrityKinematicsMeasurement::set_joint_state(
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& encoders) {
  encoders_ = encoders;
}

void TensegrityKinematicsMeasurement::set_joint_state_velocity(
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& d_encoders) {
  d_encoders_ = d_encoders;
}

Eigen::Matrix<double, 3, Eigen::Dynamic> TensegrityKinematicsMeasurement::get_J(
    int leg) const {
  int dim = encoders_.rows() / contacts_.rows();
  return jacobians_.block(0, leg * dim, 3, dim);
}

bool TensegrityKinematicsMeasurement::get_contact(int leg) const {
  return contacts_[leg];
}

double TensegrityKinematicsMeasurement::get_joint_state(int encoder) const {
  return encoders_[encoder];
}

Eigen::Matrix<double, 3, 1> TensegrityKinematicsMeasurement::get_kin_pos(
    int leg) const {
  return positions_.col(leg);
}

const Eigen::Vector3d TensegrityKinematicsMeasurement::get_init_velocity(
    const Eigen::Vector3d& w) {
  return Eigen::Vector3d::Zero();
}
}    // namespace measurement
