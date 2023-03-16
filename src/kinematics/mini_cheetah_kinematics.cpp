/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#include "kinematics/mini_cheetah_kinematics.h"

#define NLEG 4    // number of legs
#define NAPL 3    // number of actuators per leg

using namespace mini_cheetah_kinematics;

MiniCheetahKinematics::MiniCheetahKinematics() {
  position_.setConstant(3, NLEG, 0);
  jacobian_.setConstant(3, NLEG * NAPL, 0);
  contacts_.setConstant(NLEG, 1, 0);
  encoders_.setConstant(NLEG * NAPL, 1, 0);
}

MiniCheetahKinematics::MiniCheetahKinematics(
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& encoders,
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& d_encoders,
    const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts)
    : LeggedKinematicsMeasurement(encoders, d_encoders, contacts) {
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

const Eigen::Vector3d MiniCheetahKinematics::get_init_velocity(
    const Eigen::Vector3d& w) {
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  std::cout << this->get_contact(FR) << this->get_contact(FL)
            << this->get_contact(HR) << this->get_contact(HL) << std::endl;

  if (this->get_contact(FR) == 1) {
    Eigen::Vector3d pRF = p_Body_to_FrontRightFoot(encoders_);    // {I}_p_{IRF}
    Eigen::Matrix<double, 3, 12> J_pRF = Jp_Body_to_FrontRightFoot(encoders_);
    velocity = -J_pRF * d_encoders_ - lie_group::skew(w) * pRF;    // {I}_v_{WI}
  } else if (this->get_contact(FL) == 1) {
    Eigen::Vector3d pLF = p_Body_to_FrontLeftFoot(encoders_);    // {I}_p_{ILF}
    Eigen::Matrix<double, 3, 12> J_pLF = Jp_Body_to_FrontLeftFoot(encoders_);
    velocity = -J_pLF * d_encoders_ - lie_group::skew(w) * pLF;    // {I}_v_{WI}
  } else if (this->get_contact(HR) == 1) {
    Eigen::Vector3d pRH = p_Body_to_HindRightFoot(encoders_);    // {I}_p_{IRH}
    Eigen::Matrix<double, 3, 12> J_pRH = Jp_Body_to_HindRightFoot(encoders_);
    velocity = -J_pRH * d_encoders_ - lie_group::skew(w) * pRH;    // {I}_v_{WI}
  } else if (this->get_contact(HL) == 1) {
    Eigen::Vector3d pLH = p_Body_to_HindLeftFoot(encoders_);    // {I}_p_{ILH}
    Eigen::Matrix<double, 3, 12> J_pLH = Jp_Body_to_HindLeftFoot(encoders_);
    velocity = -J_pLH * d_encoders_ - lie_group::skew(w) * pLH;    // {I}_v_{WI}
  }
  return velocity;
}

int MiniCheetahKinematics::get_num_legs() { return NLEG; }