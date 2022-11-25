/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   inekf.h
 *  @author Ross Hartley
 *  @brief  Header file for Invariant EKF
 *  @date   September 25, 2018
 **/

#ifndef INEKF_INEKF_H
#define INEKF_INEKF_H
#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
#include <map>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>
#include "filter/noise_params.h"
#include "filter/observations.h"
#include "math/lie_group.h"
#include "state/robot_state.h"
#include "utils/utils.h"

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
 * @param[in] state: Robot state
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
 * @param[in] state: Robot state
 * @return None
 */
void CorrectLeftInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H,
                          const Eigen::MatrixXd& N, RobotState& state,
                          ErrorType error_type);
// void CorrectFullState(const Observation& obs); // TODO


}    // namespace inekf
#endif    // end INEKF_INEKF_H