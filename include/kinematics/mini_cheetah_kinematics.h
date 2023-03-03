/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   mini_cheetah_kinematics.h
 *  @author Justin Yu
 *  @brief  Header file for Mini Cheetah specific kinematics solver and
 *measurement container
 *  @date   January 11, 2023
 **/

#ifndef MC_KIN_H
#define MC_KIN_H

#include "kinematics/robots/mini_cheetah/Jp_Body_to_FrontLeftFoot.h"
#include "kinematics/robots/mini_cheetah/Jp_Body_to_FrontRightFoot.h"
#include "kinematics/robots/mini_cheetah/Jp_Body_to_HindLeftFoot.h"
#include "kinematics/robots/mini_cheetah/Jp_Body_to_HindRightFoot.h"
#include "kinematics/robots/mini_cheetah/p_Body_to_FrontLeftFoot.h"
#include "kinematics/robots/mini_cheetah/p_Body_to_FrontRightFoot.h"
#include "kinematics/robots/mini_cheetah/p_Body_to_HindLeftFoot.h"
#include "kinematics/robots/mini_cheetah/p_Body_to_HindRightFoot.h"
#include "measurement/legged_kinematics.h"

namespace mini_cheetah_kinematics {
enum Leg { FR, FL, HR, HL };
}

using namespace mini_cheetah_kinematics;

class MiniCheetahKinematics : public LeggedKinematicsMeasurement {
 public:
  MiniCheetahKinematics();
  MiniCheetahKinematics(
      const Eigen::Matrix<double, Eigen::Dynamic, 1>& encoders,
      const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts);

  void ComputeKinematics() override;
  int get_num_legs() override;
};

#endif    // MC_KIN_H