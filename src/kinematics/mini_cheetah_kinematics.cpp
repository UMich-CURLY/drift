/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#include "kinematics/mini_cheetah_kinematics.h"

#define NLEG 4    // number of legs
#define NAPL 3    // number of actuators per leg

MiniCheetahKinematics::MiniCheetahKinematics() {
  position_.setConstant(3, NLEG, 0);
  jacobian_.setConstant(3, NLEG * NAPL, 0);
  contacts_.setConstant(NLEG, 1, 0);
  encoders_.setConstant(NLEG * NAPL, 1, 0);
}

MiniCheetahKinematics::MiniCheetahKinematics(
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& encoders,
    const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts)
    : LeggedKinematics(encoders, contacts) {
  position_.setConstant(3, NLEG, 0);
  jacobian_.setConstant(3, NLEG * NAPL, 0);
}

void MiniCheetahKinematics::ComputeKinematics() {
  position_.col(FR) = p_Body_to_FrontRightFoot(encoders_);
  position_.col(FL) = p_Body_to_FrontLeftFoot(encoders_);
  position_.col(HR) = p_Body_to_HindRightFoot(encoders_);
  position_.col(HL) = p_Body_to_HindLeftFoot(encoders_);
  jacobian_.block(0, FR * NAPL, 3, NAPL)
      = Jp_Body_to_FrontRightFoot(encoders_).block(0, FR * NAPL, 3, NAPL);
  jacobian_.block(0, FL * NAPL, 3, NAPL)
      = Jp_Body_to_FrontLeftFoot(encoders_).block(0, FL * NAPL, 3, NAPL);
  jacobian_.block(0, HR * NAPL, 3, NAPL)
      = Jp_Body_to_HindRightFoot(encoders_).block(0, HR * NAPL, 3, NAPL);
  jacobian_.block(0, HL * NAPL, 3, NAPL)
      = Jp_Body_to_HindLeftFoot(encoders_).block(0, HL * NAPL, 3, NAPL);
}

int MiniCheetahKinematics::get_num_legs() { return NLEG; }