/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   InEKF.cpp
 *  @author Ross Hartley
 *  @brief  Source file for Invariant EKF
 *  @date   November 25, 2022
 **/

#include "filter/inekf/inekf.h"

namespace inekf {

using namespace std;
using namespace lie_group;


// Correct Input State: Right-Invariant Observation
void CorrectRightInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H,
                           const Eigen::MatrixXd& N, RobotState& state,
                           ErrorType error_type) {
  // Get current state estimate
  Eigen::MatrixXd X = state.get_X();
  Eigen::VectorXd Theta = state.get_theta();
  Eigen::MatrixXd P = state.get_P();
  int dimX = state.dimX();
  int dimTheta = state.dimTheta();
  int dimP = state.dimP();

  // Remove bias
  Theta = Eigen::Matrix<double, 6, 1>::Zero();
  P.block<6, 6>(dimP - dimTheta, dimP - dimTheta)
      = 0.0001 * Eigen::Matrix<double, 6, 6>::Identity();
  P.block(0, dimP - dimTheta, dimP - dimTheta, dimTheta)
      = Eigen::MatrixXd::Zero(dimP - dimTheta, dimTheta);
  P.block(dimP - dimTheta, 0, dimTheta, dimP - dimTheta)
      = Eigen::MatrixXd::Zero(dimTheta, dimP - dimTheta);

  // Map from left invariant to right invariant error temporarily
  if (error_type == ErrorType::LeftInvariant) {
    Eigen::MatrixXd Adj = Eigen::MatrixXd::Identity(dimP, dimP);
    Adj.block(0, 0, dimP - dimTheta, dimP - dimTheta) = Adjoint_SEK3(X);
    P = (Adj * P * Adj.transpose()).eval();
  }

  // Compute Kalman Gain
  Eigen::MatrixXd PHT = P * H.transpose();
  Eigen::MatrixXd S = H * PHT + N;
  Eigen::MatrixXd K = PHT * S.inverse();

  // Compute state correction vector
  Eigen::VectorXd delta = K * Z;
  Eigen::MatrixXd dX = Exp_SEK3(delta.segment(0, delta.rows() - dimTheta));
  Eigen::VectorXd dTheta = delta.segment(delta.rows() - dimTheta, dimTheta);

  // Update state
  Eigen::MatrixXd X_new = dX * X;    // Right-Invariant Update
  Eigen::VectorXd Theta_new = Theta + dTheta;

  // Set new state
  state.set_X(X_new);
  state.set_theta(Theta_new);

  // Update Covariance
  Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(dimP, dimP) - K * H;
  Eigen::MatrixXd P_new = IKH * P * IKH.transpose()
                          + K * N * K.transpose();    // Joseph update form

  // Map from right invariant back to left invariant error
  if (error_type == ErrorType::LeftInvariant) {
    Eigen::MatrixXd AdjInv = Eigen::MatrixXd::Identity(dimP, dimP);
    AdjInv.block(0, 0, dimP - dimTheta, dimP - dimTheta)
        = Adjoint_SEK3(state.Xinv());
    P_new = (AdjInv * P_new * AdjInv.transpose()).eval();
  }

  // Set new covariance
  state.set_P(P_new);
}


// Correct Input State: Left-Invariant Observation
void CorrectLeftInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H,
                          const Eigen::MatrixXd& N, RobotState& state,
                          ErrorType error_type) {
  // Get current state estimate
  Eigen::MatrixXd X = state.get_X();
  Eigen::VectorXd Theta = state.get_theta();
  Eigen::MatrixXd P = state.get_P();
  int dimX = state.dimX();
  int dimTheta = state.dimTheta();
  int dimP = state.dimP();

  // Map from right invariant to left invariant error temporarily
  if (error_type == ErrorType::RightInvariant) {
    Eigen::MatrixXd AdjInv = Eigen::MatrixXd::Identity(dimP, dimP);
    AdjInv.block(0, 0, dimP - dimTheta, dimP - dimTheta)
        = Adjoint_SEK3(state.Xinv());
    P = (AdjInv * P * AdjInv.transpose()).eval();
  }

  // Compute Kalman Gain
  Eigen::MatrixXd PHT = P * H.transpose();
  Eigen::MatrixXd S = H * PHT + N;
  Eigen::MatrixXd K = PHT * S.inverse();

  // Compute state correction vector
  Eigen::VectorXd delta = K * Z;
  Eigen::MatrixXd dX = Exp_SEK3(delta.segment(0, delta.rows() - dimTheta));
  Eigen::VectorXd dTheta = delta.segment(delta.rows() - dimTheta, dimTheta);

  // Update state
  Eigen::MatrixXd X_new = X * dX;    // Left-Invariant Update
  Eigen::VectorXd Theta_new = Theta + dTheta;

  // Set new state
  state.set_X(X_new);
  state.set_theta(Theta_new);

  // Update Covariance
  Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(dimP, dimP) - K * H;
  Eigen::MatrixXd P_new = IKH * P * IKH.transpose()
                          + K * N * K.transpose();    // Joseph update form

  // Map from left invariant back to right invariant error
  if (error_type == ErrorType::RightInvariant) {
    Eigen::MatrixXd Adj = Eigen::MatrixXd::Identity(dimP, dimP);
    Adj.block(0, 0, dimP - dimTheta, dimP - dimTheta) = Adjoint_SEK3(X_new);
    P_new = (Adj * P_new * Adj.transpose()).eval();
  }

  // Set new covariance
  state.set_P(P_new);
}

}    // namespace inekf