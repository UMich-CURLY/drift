/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   go1_kinematics.h
 *  @author Wenzhe Tong
 *  @brief  Header file for GO1 specific kinematics solver and
 *measurement container
 *  @date   Feb 24, 2023
 **/

#ifndef GO1_KIN_H
#define GO1_KIN_H

#include "kinematics/robots/go1/Jp_Body_to_FrontLeftFoot.h"
#include "kinematics/robots/go1/Jp_Body_to_FrontRightFoot.h"
#include "kinematics/robots/go1/Jp_Body_to_HindLeftFoot.h"
#include "kinematics/robots/go1/Jp_Body_to_HindRightFoot.h"
#include "kinematics/robots/go1/p_Body_to_FrontLeftFoot.h"
#include "kinematics/robots/go1/p_Body_to_FrontRightFoot.h"
#include "kinematics/robots/go1/p_Body_to_HindLeftFoot.h"
#include "kinematics/robots/go1/p_Body_to_HindRightFoot.h"
#include "measurement/legged_kinematics.h"

enum Leg { FR, FL, HR, HL };

class GO1Kinematics : public LeggedKinematicsMeasurement {
 public:
  GO1Kinematics();
  GO1Kinematics(
      const Eigen::Matrix<double, Eigen::Dynamic, 1>& encoders,
      const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts);

  void ComputeKinematics() override;
  int get_num_legs() override;
};

#endif    // GO1_KIN_H