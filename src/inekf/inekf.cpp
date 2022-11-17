/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   InEKF.cpp
 *  @author Ross Hartley
 *  @brief  Source file for Invariant EKF
 *  @date   September 25, 2018
 **/

#include "inekf/inekf.h"

namespace inekf {

using namespace std;
using namespace lie_group;

// Default constructor
InEKF::InEKF()
    : g_((Eigen::VectorXd(3) << 0, 0, -9.81).finished()), magnetic_field_((Eigen::VectorXd(3) << 0, 0, 0).finished()) {}

// Constructor with noise params
InEKF::InEKF(NoiseParams params)
    : g_((Eigen::VectorXd(3) << 0, 0, -9.81).finished()),
      magnetic_field_((Eigen::VectorXd(3) << std::cos(1.2049), 0, std::sin(1.2049)).finished()),
      noise_params_(params) {}

// Constructor with initial state
InEKF::InEKF(RobotState state)
    : g_((Eigen::VectorXd(3) << 0, 0, -9.81).finished()),
      magnetic_field_((Eigen::VectorXd(3) << std::cos(1.2049), 0, std::sin(1.2049)).finished()),
      state_(state) {}

// Constructor with error type
InEKF::InEKF(ErrorType error_type)
    : g_((Eigen::VectorXd(3) << 0, 0, -9.81).finished()),
      magnetic_field_((Eigen::VectorXd(3) << 0, 0, 0).finished()),
      error_type_(error_type) {}

// Constructor with initial state and noise params
InEKF::InEKF(RobotState state, NoiseParams params)
    : g_((Eigen::VectorXd(3) << 0, 0, -9.81).finished()),
      magnetic_field_((Eigen::VectorXd(3) << std::cos(1.2049), 0, std::sin(1.2049)).finished()),
      state_(state),
      noise_params_(params) {}

// Constructor with noise params and error type
InEKF::InEKF(NoiseParams params, ErrorType error_type)
    : g_((Eigen::VectorXd(3) << 0, 0, -9.81).finished()),
      magnetic_field_((Eigen::VectorXd(3) << std::cos(1.2049), 0, std::sin(1.2049)).finished()),
      noise_params_(params),
      error_type_(error_type) {}

// Constructor with initial state, noise params, and error type
InEKF::InEKF(RobotState state, NoiseParams params, ErrorType error_type)
    : g_((Eigen::VectorXd(3) << 0, 0, -9.81).finished()),
      magnetic_field_((Eigen::VectorXd(3) << std::cos(1.2049), 0, std::sin(1.2049)).finished()),
      state_(state),
      noise_params_(params),
      error_type_(error_type) {}

// Clear all data in the filter
void InEKF::clear() {
    state_ = RobotState();
    noise_params_ = NoiseParams();
}

// Returns the robot's current error type
ErrorType InEKF::get_error_type() const { return error_type_; }

// Return robot's current state
RobotState InEKF::get_state() const { return state_; }

// Sets the robot's current state
void InEKF::set_state(RobotState state) { state_ = state; }


// Correct State: Right-Invariant Observation
void InEKF::CorrectRightInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& N) {
    // Get current state estimate
    Eigen::MatrixXd X = state_.getX();
    Eigen::VectorXd Theta = state_.getTheta();
    Eigen::MatrixXd P = state_.getP();
    int dimX = state_.dimX();
    int dimTheta = state_.dimTheta();
    int dimP = state_.dimP();

    // Remove bias
    Theta = Eigen::Matrix<double, 6, 1>::Zero();
    P.block<6, 6>(dimP - dimTheta, dimP - dimTheta) = 0.0001 * Eigen::Matrix<double, 6, 6>::Identity();
    P.block(0, dimP - dimTheta, dimP - dimTheta, dimTheta) = Eigen::MatrixXd::Zero(dimP - dimTheta, dimTheta);
    P.block(dimP - dimTheta, 0, dimTheta, dimP - dimTheta) = Eigen::MatrixXd::Zero(dimTheta, dimP - dimTheta);

    // Map from left invariant to right invariant error temporarily
    if (error_type_ == ErrorType::LeftInvariant) {
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
    state_.setX(X_new);
    state_.setTheta(Theta_new);

    // Update Covariance
    Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(dimP, dimP) - K * H;
    Eigen::MatrixXd P_new = IKH * P * IKH.transpose() + K * N * K.transpose();    // Joseph update form

    // Map from right invariant back to left invariant error
    if (error_type_ == ErrorType::LeftInvariant) {
        Eigen::MatrixXd AdjInv = Eigen::MatrixXd::Identity(dimP, dimP);
        AdjInv.block(0, 0, dimP - dimTheta, dimP - dimTheta) = Adjoint_SEK3(state_.Xinv());
        P_new = (AdjInv * P_new * AdjInv.transpose()).eval();
    }

    // Set new covariance
    state_.setP(P_new);
}

// Correct Input State: Right-Invariant Observation
void InEKF::CorrectRightInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& N,
                                  RobotState& state) {
    // Get current state estimate
    Eigen::MatrixXd X = state.getX();
    Eigen::VectorXd Theta = state.getTheta();
    Eigen::MatrixXd P = state.getP();
    int dimX = state.dimX();
    int dimTheta = state.dimTheta();
    int dimP = state.dimP();

    // Remove bias
    Theta = Eigen::Matrix<double, 6, 1>::Zero();
    P.block<6, 6>(dimP - dimTheta, dimP - dimTheta) = 0.0001 * Eigen::Matrix<double, 6, 6>::Identity();
    P.block(0, dimP - dimTheta, dimP - dimTheta, dimTheta) = Eigen::MatrixXd::Zero(dimP - dimTheta, dimTheta);
    P.block(dimP - dimTheta, 0, dimTheta, dimP - dimTheta) = Eigen::MatrixXd::Zero(dimTheta, dimP - dimTheta);
    // std::cout << "P:\n" << P << std::endl;
    // std::cout << state << std::endl;

    // Map from left invariant to right invariant error temporarily
    if (error_type_ == ErrorType::LeftInvariant) {
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
    std::cout << "dX:\n" << dX << std::endl;
    Eigen::VectorXd dTheta = delta.segment(delta.rows() - dimTheta, dimTheta);

    // Update state
    Eigen::MatrixXd X_new = dX * X;    // Right-Invariant Update
    Eigen::VectorXd Theta_new = Theta + dTheta;

    // Set new state
    state.setX(X_new);
    state.setTheta(Theta_new);

    // Update Covariance
    Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(dimP, dimP) - K * H;
    Eigen::MatrixXd P_new = IKH * P * IKH.transpose() + K * N * K.transpose();    // Joseph update form

    // Map from right invariant back to left invariant error
    if (error_type_ == ErrorType::LeftInvariant) {
        Eigen::MatrixXd AdjInv = Eigen::MatrixXd::Identity(dimP, dimP);
        AdjInv.block(0, 0, dimP - dimTheta, dimP - dimTheta) = Adjoint_SEK3(state_.Xinv());
        P_new = (AdjInv * P_new * AdjInv.transpose()).eval();
    }

    // Set new covariance
    state.setP(P_new);
}

// Correct State: Left-Invariant Observation
void InEKF::CorrectLeftInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& N) {
    // Get current state estimate
    Eigen::MatrixXd X = state_.getX();
    Eigen::VectorXd Theta = state_.getTheta();
    Eigen::MatrixXd P = state_.getP();
    int dimX = state_.dimX();
    int dimTheta = state_.dimTheta();
    int dimP = state_.dimP();

    // Map from right invariant to left invariant error temporarily
    if (error_type_ == ErrorType::RightInvariant) {
        Eigen::MatrixXd AdjInv = Eigen::MatrixXd::Identity(dimP, dimP);
        AdjInv.block(0, 0, dimP - dimTheta, dimP - dimTheta) = Adjoint_SEK3(state_.Xinv());
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
    state_.setX(X_new);
    state_.setTheta(Theta_new);

    // Update Covariance
    Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(dimP, dimP) - K * H;
    Eigen::MatrixXd P_new = IKH * P * IKH.transpose() + K * N * K.transpose();    // Joseph update form

    // Map from left invariant back to right invariant error
    if (error_type_ == ErrorType::RightInvariant) {
        Eigen::MatrixXd Adj = Eigen::MatrixXd::Identity(dimP, dimP);
        Adj.block(0, 0, dimP - dimTheta, dimP - dimTheta) = Adjoint_SEK3(X_new);
        P_new = (Adj * P_new * Adj.transpose()).eval();
    }

    // Set new covariance
    state_.setP(P_new);
}

// Correct Input State: Left-Invariant Observation
void InEKF::CorrectLeftInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& N,
                                 RobotState& state) {
    // Get current state estimate
    Eigen::MatrixXd X = state.getX();
    Eigen::VectorXd Theta = state.getTheta();
    Eigen::MatrixXd P = state.getP();
    int dimX = state.dimX();
    int dimTheta = state.dimTheta();
    int dimP = state.dimP();

    // Map from right invariant to left invariant error temporarily
    if (error_type_ == ErrorType::RightInvariant) {
        Eigen::MatrixXd AdjInv = Eigen::MatrixXd::Identity(dimP, dimP);
        AdjInv.block(0, 0, dimP - dimTheta, dimP - dimTheta) = Adjoint_SEK3(state_.Xinv());
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
    state.setX(X_new);
    state.setTheta(Theta_new);

    // Update Covariance
    Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(dimP, dimP) - K * H;
    Eigen::MatrixXd P_new = IKH * P * IKH.transpose() + K * N * K.transpose();    // Joseph update form

    // Map from left invariant back to right invariant error
    if (error_type_ == ErrorType::RightInvariant) {
        Eigen::MatrixXd Adj = Eigen::MatrixXd::Identity(dimP, dimP);
        Adj.block(0, 0, dimP - dimTheta, dimP - dimTheta) = Adjoint_SEK3(X_new);
        P_new = (Adj * P_new * Adj.transpose()).eval();
    }

    // Set new covariance
    state.setP(P_new);
}

}    // namespace inekf