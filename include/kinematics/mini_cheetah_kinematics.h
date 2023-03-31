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
#include "math/lie_group.h"
#include "measurement/legged_kinematics.h"

namespace mini_cheetah_kinematics {
enum Leg { FR, FL, HR, HL };
}

using namespace mini_cheetah_kinematics;

class MiniCheetahKinematics : public LeggedKinematicsMeasurement {
 public:
  /// @name Constructor
  /// @{
  /**
   * @brief Default constructor. Will generate an empty measurement.
   */
  MiniCheetahKinematics();

  /**
   * @brief Constructor with encoder and contact information
   * @param[in] encoders Joint encoder values
   * @param[in] d_encoders Joint encoder velocity values
   * @param[in] contacts Contact information
   */
  MiniCheetahKinematics(
      const Eigen::Matrix<double, Eigen::Dynamic, 1>& encoders,
      const Eigen::Matrix<double, Eigen::Dynamic, 1>& d_encoders,
      const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts);

  /// @}

  /**
   * @brief Compute kinematics and store in measurement
   */
  void ComputeKinematics() override;

  /**
   * @brief Get number of legs
   * @return Number of legs
   */
  int get_num_legs() override;

  /**
   * @brief Get initial velocity of the robot based on encoder values and
   * initial angular velocity
   * @param[in] w Initial angular velocity of the robot (rad/s)
   * @return Initial velocity
   */
  const Eigen::Vector3d get_init_velocity(const Eigen::Vector3d& w) override;
};

#endif    // MC_KIN_H