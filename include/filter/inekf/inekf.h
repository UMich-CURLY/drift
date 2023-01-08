/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   inekf.h
 *  @author Tingjun Li, Ross Hartley
 *  @brief  Header file for Invariant EKF.
 *  Original paper:
 *  https://journals.sagepub.com/doi/full/10.1177/0278364919894385
 *  Original github repo:
 *  https://github.com/RossHartley/invariant-ekf
 *
 *  @date   November 25, 2022
 **/

#ifndef INEKF_INEKF_H
#define INEKF_INEKF_H
#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
#include <map>
#include <vector>

#include "math/lie_group.h"
#include "state/robot_state.h"

namespace inekf {

enum ErrorType { LeftInvariant, RightInvariant };

using ContactState = std::pair<int, bool>;


/// @}
// ======================================================================
/**
 * @brief Corrects the state using Right Invariant observation model with
 * given measurement, output and matrices.
 *
 * @param[in] Z: innovation matrix
 * @param[in] H: measurement error matrix
 * @param[in] N: measurement noise matrix
 * @param[in/out] state: Robot state
 * @return None
 */
void CorrectRightInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H,
                           const Eigen::MatrixXd& N, RobotState& state,
                           ErrorType error_type);

// ======================================================================
/**
 * @brief Corrects the state using Left Invariant observation model with
 * given measurement, output and matrices.
 *
 * @param[in] Z: innovation matrix
 * @param[in] H: measurement error matrix
 * @param[in] N: measurement noise matrix
 * @param[in/out] state: Robot state
 * @return None
 */
void CorrectLeftInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H,
                          const Eigen::MatrixXd& N, RobotState& state,
                          ErrorType error_type);


}    // namespace inekf
#endif    // end INEKF_INEKF_H