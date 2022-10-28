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

void removeRowAndColumn(Eigen::MatrixXd& M, int index);

// Default constructor
InEKF::InEKF() : 
    g_((Eigen::VectorXd(3) << 0,0,-9.81).finished()), 
    magnetic_field_((Eigen::VectorXd(3) << 0,0,0).finished()) {}

// Constructor with noise params
InEKF::InEKF(NoiseParams params) : 
    g_((Eigen::VectorXd(3) << 0,0,-9.81).finished()), 
    magnetic_field_((Eigen::VectorXd(3) << std::cos(1.2049),0,std::sin(1.2049)).finished()), 
    noise_params_(params) {}

// Constructor with initial state
InEKF::InEKF(RobotState state) : 
    g_((Eigen::VectorXd(3) << 0,0,-9.81).finished()), 
    magnetic_field_((Eigen::VectorXd(3) << std::cos(1.2049),0,std::sin(1.2049)).finished()), 
    state_(state) {}

// Constructor with initial state and noise params
InEKF::InEKF(RobotState state, NoiseParams params) : 
    g_((Eigen::VectorXd(3) << 0,0,-9.81).finished()), 
    magnetic_field_((Eigen::VectorXd(3) << std::cos(1.2049),0,std::sin(1.2049)).finished()), 
    state_(state), 
    noise_params_(params) {}

// Constructor with initial state, noise params, and error type
InEKF::InEKF(RobotState state, NoiseParams params, ErrorType error_type) : 
    g_((Eigen::VectorXd(3) << 0,0,-9.81).finished()), 
    magnetic_field_((Eigen::VectorXd(3) << std::cos(1.2049),0,std::sin(1.2049)).finished()), 
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

// Return noise params
NoiseParams InEKF::get_noise_params() const { return noise_params_; }

// Sets the filter's noise parameters
void InEKF::set_noise_params(NoiseParams params) { noise_params_ = params; }

// Compute Analytical state transition matrix
Eigen::MatrixXd InEKF::StateTransitionMatrix(Eigen::Vector3d& w, Eigen::Vector3d& a, double dt) {
    Eigen::Vector3d phi = w*dt;
    Eigen::Matrix3d G0 = Gamma_SO3(phi,0); // Computation can be sped up by computing G0,G1,G2 all at once
    Eigen::Matrix3d G1 = Gamma_SO3(phi,1); // TODO: These are also needed for the mean propagation, we should not compute twice
    Eigen::Matrix3d G2 = Gamma_SO3(phi,2);
    Eigen::Matrix3d G0t = G0.transpose();
    Eigen::Matrix3d G1t = G1.transpose();
    Eigen::Matrix3d G2t = G2.transpose();
    Eigen::Matrix3d G3t = Gamma_SO3(-phi,3);
    
    int dimX = state_.dimX();
    int dimTheta = state_.dimTheta();
    int dimP = state_.dimP();
    Eigen::MatrixXd Phi = Eigen::MatrixXd::Identity(dimP,dimP);

    // Compute the complicated bias terms (derived for the left invariant case)
    Eigen::Matrix3d ax = skew(a);
    Eigen::Matrix3d wx = skew(w);
    Eigen::Matrix3d wx2 = wx*wx;
    double dt2 = dt*dt;
    double dt3 = dt2*dt;
    double theta = w.norm();
    double theta2 = theta*theta;
    double theta3 = theta2*theta;
    double theta4 = theta3*theta;
    double theta5 = theta4*theta;
    double theta6 = theta5*theta;
    double theta7 = theta6*theta;
    double thetadt = theta*dt;
    double thetadt2 = thetadt*thetadt;
    double thetadt3 = thetadt2*thetadt;
    double sinthetadt = sin(thetadt);
    double costhetadt = cos(thetadt);
    double sin2thetadt = sin(2*thetadt);
    double cos2thetadt = cos(2*thetadt);
    double thetadtcosthetadt = thetadt*costhetadt;
    double thetadtsinthetadt = thetadt*sinthetadt;

    Eigen::Matrix3d Phi25L = G0t*(ax*G2t*dt2 
        + ((sinthetadt-thetadtcosthetadt)/(theta3))*(wx*ax)
        - ((cos2thetadt-4*costhetadt+3)/(4*theta4))*(wx*ax*wx)
        + ((4*sinthetadt+sin2thetadt-4*thetadtcosthetadt-2*thetadt)/(4*theta5))*(wx*ax*wx2)
        + ((thetadt2-2*thetadtsinthetadt-2*costhetadt+2)/(2*theta4))*(wx2*ax)
        - ((6*thetadt-8*sinthetadt+sin2thetadt)/(4*theta5))*(wx2*ax*wx)
        + ((2*thetadt2-4*thetadtsinthetadt-cos2thetadt+1)/(4*theta6))*(wx2*ax*wx2) );

    Eigen::Matrix3d Phi35L = G0t*(ax*G3t*dt3
        - ((thetadtsinthetadt+2*costhetadt-2)/(theta4))*(wx*ax)
        - ((6*thetadt-8*sinthetadt+sin2thetadt)/(8*theta5))*(wx*ax*wx)
        - ((2*thetadt2+8*thetadtsinthetadt+16*costhetadt+cos2thetadt-17)/(8*theta6))*(wx*ax*wx2)
        + ((thetadt3+6*thetadt-12*sinthetadt+6*thetadtcosthetadt)/(6*theta5))*(wx2*ax)
        - ((6*thetadt2+16*costhetadt-cos2thetadt-15)/(8*theta6))*(wx2*ax*wx)
        + ((4*thetadt3+6*thetadt-24*sinthetadt-3*sin2thetadt+24*thetadtcosthetadt)/(24*theta7))*(wx2*ax*wx2) );

    
    // TODO: Get better approximation using taylor series when theta < tol
    const double tol =  1e-6;
    if (theta < tol) {
        Phi25L = (1/2)*ax*dt2;
        Phi35L = (1/6)*ax*dt3;
    }

    // Fill out analytical state transition matrices
    if  ((state_.getStateType() == StateType::WorldCentric && error_type_ == ErrorType::LeftInvariant) || 
         (state_.getStateType() == StateType::BodyCentric && error_type_ == ErrorType::RightInvariant)) {
        // Compute left-invariant state transisition matrix
        Phi.block<3,3>(0,0) = G0t; // Phi_11
        Phi.block<3,3>(3,0) = -G0t*skew(G1*a)*dt; // Phi_21
        Phi.block<3,3>(6,0) = -G0t*skew(G2*a)*dt2; // Phi_31
        Phi.block<3,3>(3,3) = G0t; // Phi_22
        Phi.block<3,3>(6,3) = G0t*dt; // Phi_32
        Phi.block<3,3>(6,6) = G0t; // Phi_33
        for (int i=5; i<dimX; ++i) {
            Phi.block<3,3>((i-2)*3,(i-2)*3) = G0t; // Phi_(3+i)(3+i)
        }
        Phi.block<3,3>(0,dimP-dimTheta) = -G1t*dt; // Phi_15
        Phi.block<3,3>(3,dimP-dimTheta) = Phi25L; // Phi_25
        Phi.block<3,3>(6,dimP-dimTheta) = Phi35L; // Phi_35
        Phi.block<3,3>(3,dimP-dimTheta+3) = -G1t*dt; // Phi_26
        Phi.block<3,3>(6,dimP-dimTheta+3) = -G0t*G2*dt2; // Phi_36
    } else {
        // Compute right-invariant state transition matrix (Assumes unpropagated state)
        Eigen::Matrix3d gx = skew(g_);
        Eigen::Matrix3d R = state_.getRotation();
        Eigen::Vector3d v = state_.getVelocity();
        Eigen::Vector3d p = state_.getPosition();
        Eigen::Matrix3d RG0 = R*G0;
        Eigen::Matrix3d RG1dt = R*G1*dt;
        Eigen::Matrix3d RG2dt2 = R*G2*dt2;
        Phi.block<3,3>(3,0) = gx*dt; // Phi_21
        Phi.block<3,3>(6,0) = 0.5*gx*dt2; // Phi_31
        Phi.block<3,3>(6,3) = Eigen::Matrix3d::Identity()*dt; // Phi_32
        Phi.block<3,3>(0,dimP-dimTheta) = -RG1dt; // Phi_15
        Phi.block<3,3>(3,dimP-dimTheta) = -skew(v+RG1dt*a+g_*dt)*RG1dt + RG0*Phi25L; // Phi_25
        Phi.block<3,3>(6,dimP-dimTheta) = -skew(p+v*dt+RG2dt2*a+0.5*g_*dt2)*RG1dt + RG0*Phi35L; // Phi_35
        for (int i=5; i<dimX; ++i) {
            Phi.block<3,3>((i-2)*3,dimP-dimTheta) = -skew(state_.getVector(i))*RG1dt; // Phi_(3+i)5
        }
        Phi.block<3,3>(3,dimP-dimTheta+3) = -RG1dt; // Phi_26
        Phi.block<3,3>(6,dimP-dimTheta+3) = -RG2dt2; // Phi_36
    }
    return Phi;
}


// Compute Discrete noise matrix
Eigen::MatrixXd InEKF::DiscreteNoiseMatrix(Eigen::MatrixXd& Phi, double dt, const std::map<int,int>& augmented_states){
    int dimX = state_.dimX();
    int dimTheta = state_.dimTheta();
    int dimP = state_.dimP();    
    Eigen::MatrixXd G = Eigen::MatrixXd::Identity(dimP,dimP);

    // Compute G using Adjoint of Xk if needed, otherwise identity (Assumes unpropagated state)
    if  ((state_.getStateType() == StateType::WorldCentric && error_type_ == ErrorType::RightInvariant) || 
         (state_.getStateType() == StateType::BodyCentric && error_type_ == ErrorType::LeftInvariant)) {
        G.block(0,0,dimP-dimTheta,dimP-dimTheta) = Adjoint_SEK3(state_.getWorldX()); 
    }

    // Continuous noise covariance 
    Eigen::MatrixXd Qc = Eigen::MatrixXd::Zero(dimP,dimP); // Landmark noise terms will remain zero
    Qc.block<3,3>(0,0) = noise_params_.getGyroscopeCov(); 
    Qc.block<3,3>(3,3) = noise_params_.getAccelerometerCov();

    for(map<int,int>::const_iterator it=augmented_states.begin(); it!=augmented_states.end(); ++it) {
        Qc.block<3,3>(3+3*(it->second-3),3+3*(it->second-3)) = noise_params_.getAugmentCov(); // Augment state noise terms
    } // TODO: Use kinematic orientation to map noise from augment state frame to body frame (not needed if noise is isotropic)
    
    Qc.block<3,3>(dimP-dimTheta,dimP-dimTheta) = noise_params_.getGyroscopeBiasCov();
    Qc.block<3,3>(dimP-dimTheta+3,dimP-dimTheta+3) = noise_params_.getAccelerometerBiasCov();

    // Noise Covariance Discretization
    Eigen::MatrixXd PhiG = Phi * G;
    Eigen::MatrixXd Qd = PhiG * Qc * PhiG.transpose() * dt; // Approximated discretized noise matrix (TODO: compute analytical)
    return Qd;
}


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
    Theta = Eigen::Matrix<double,6,1>::Zero();
    P.block<6,6>(dimP-dimTheta,dimP-dimTheta) = 0.0001*Eigen::Matrix<double,6,6>::Identity();
    P.block(0,dimP-dimTheta,dimP-dimTheta,dimTheta) = Eigen::MatrixXd::Zero(dimP-dimTheta,dimTheta);
    P.block(dimP-dimTheta,0,dimTheta,dimP-dimTheta) = Eigen::MatrixXd::Zero(dimTheta,dimP-dimTheta);
    // std::cout << "P:\n" << P << std::endl;
    // std::cout << state_ << std::endl;

    // Map from left invariant to right invariant error temporarily
    if (error_type_==ErrorType::LeftInvariant) {
        Eigen::MatrixXd Adj = Eigen::MatrixXd::Identity(dimP,dimP);
        Adj.block(0,0,dimP-dimTheta,dimP-dimTheta) = Adjoint_SEK3(X); 
        P = (Adj * P * Adj.transpose()).eval(); 
    }

    // Compute Kalman Gain
    Eigen::MatrixXd PHT = P * H.transpose();
    Eigen::MatrixXd S = H * PHT + N;
    Eigen::MatrixXd K = PHT * S.inverse();

    // Compute state correction vector
    Eigen::VectorXd delta = K*Z;
    Eigen::MatrixXd dX = Exp_SEK3(delta.segment(0,delta.rows()-dimTheta));
    Eigen::VectorXd dTheta = delta.segment(delta.rows()-dimTheta, dimTheta);

    // Update state
    Eigen::MatrixXd X_new = dX*X; // Right-Invariant Update
    Eigen::VectorXd Theta_new = Theta + dTheta;

    // Set new state  
    state_.setX(X_new); 
    state_.setTheta(Theta_new);

    // Update Covariance
    Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(dimP,dimP) - K*H;
    Eigen::MatrixXd P_new = IKH * P * IKH.transpose() + K*N*K.transpose(); // Joseph update form

    // Map from right invariant back to left invariant error
    if (error_type_==ErrorType::LeftInvariant) {
        Eigen::MatrixXd AdjInv = Eigen::MatrixXd::Identity(dimP,dimP);
        AdjInv.block(0,0,dimP-dimTheta,dimP-dimTheta) = Adjoint_SEK3(state_.Xinv()); 
        P_new = (AdjInv * P_new * AdjInv.transpose()).eval();
    }

    // Set new covariance
    state_.setP(P_new); 
}   

// Correct Input State: Right-Invariant Observation
void InEKF::CorrectRightInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& N, RobotState& state) {
    // Get current state estimate
    Eigen::MatrixXd X = state.getX();
    Eigen::VectorXd Theta = state.getTheta();
    Eigen::MatrixXd P = state.getP();
    int dimX = state.dimX();
    int dimTheta = state.dimTheta();
    int dimP = state.dimP();

    // Remove bias
    Theta = Eigen::Matrix<double,6,1>::Zero();
    P.block<6,6>(dimP-dimTheta,dimP-dimTheta) = 0.0001*Eigen::Matrix<double,6,6>::Identity();
    P.block(0,dimP-dimTheta,dimP-dimTheta,dimTheta) = Eigen::MatrixXd::Zero(dimP-dimTheta,dimTheta);
    P.block(dimP-dimTheta,0,dimTheta,dimP-dimTheta) = Eigen::MatrixXd::Zero(dimTheta,dimP-dimTheta);
    // std::cout << "P:\n" << P << std::endl;
    // std::cout << state << std::endl;

    // Map from left invariant to right invariant error temporarily
    if (error_type_==ErrorType::LeftInvariant) {
        Eigen::MatrixXd Adj = Eigen::MatrixXd::Identity(dimP,dimP);
        Adj.block(0,0,dimP-dimTheta,dimP-dimTheta) = Adjoint_SEK3(X); 
        P = (Adj * P * Adj.transpose()).eval(); 
    }

    // Compute Kalman Gain
    Eigen::MatrixXd PHT = P * H.transpose();
    Eigen::MatrixXd S = H * PHT + N;
    Eigen::MatrixXd K = PHT * S.inverse();

    // Compute state correction vector
    Eigen::VectorXd delta = K*Z;
    Eigen::MatrixXd dX = Exp_SEK3(delta.segment(0,delta.rows()-dimTheta));
    Eigen::VectorXd dTheta = delta.segment(delta.rows()-dimTheta, dimTheta);

    // Update state
    Eigen::MatrixXd X_new = dX*X; // Right-Invariant Update
    Eigen::VectorXd Theta_new = Theta + dTheta;

    // Set new state  
    state.setX(X_new); 
    state.setTheta(Theta_new);

    // Update Covariance
    Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(dimP,dimP) - K*H;
    Eigen::MatrixXd P_new = IKH * P * IKH.transpose() + K*N*K.transpose(); // Joseph update form

    // Map from right invariant back to left invariant error
    if (error_type_==ErrorType::LeftInvariant) {
        Eigen::MatrixXd AdjInv = Eigen::MatrixXd::Identity(dimP,dimP);
        AdjInv.block(0,0,dimP-dimTheta,dimP-dimTheta) = Adjoint_SEK3(state_.Xinv()); 
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
    if (error_type_==ErrorType::RightInvariant) {
        Eigen::MatrixXd AdjInv = Eigen::MatrixXd::Identity(dimP,dimP);
        AdjInv.block(0,0,dimP-dimTheta,dimP-dimTheta) = Adjoint_SEK3(state_.Xinv()); 
        P = (AdjInv * P * AdjInv.transpose()).eval();
    }

    // Compute Kalman Gain
    Eigen::MatrixXd PHT = P * H.transpose();
    Eigen::MatrixXd S = H * PHT + N;
    Eigen::MatrixXd K = PHT * S.inverse();

    // Compute state correction vector
    Eigen::VectorXd delta = K*Z;
    Eigen::MatrixXd dX = Exp_SEK3(delta.segment(0,delta.rows()-dimTheta));
    Eigen::VectorXd dTheta = delta.segment(delta.rows()-dimTheta, dimTheta);

    // Update state
    Eigen::MatrixXd X_new = X*dX; // Left-Invariant Update
    Eigen::VectorXd Theta_new = Theta + dTheta;

    // Set new state
    state_.setX(X_new); 
    state_.setTheta(Theta_new);

    // Update Covariance
    Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(dimP,dimP) - K*H;
    Eigen::MatrixXd P_new = IKH * P * IKH.transpose() + K*N*K.transpose(); // Joseph update form

    // Map from left invariant back to right invariant error
    if (error_type_==ErrorType::RightInvariant) {
        Eigen::MatrixXd Adj = Eigen::MatrixXd::Identity(dimP,dimP);
        Adj.block(0,0,dimP-dimTheta,dimP-dimTheta) = Adjoint_SEK3(X_new); 
        P_new = (Adj * P_new * Adj.transpose()).eval(); 
    }

    // Set new covariance
    state_.setP(P_new); 
}   

// Correct Input State: Left-Invariant Observation
void InEKF::CorrectLeftInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& N, RobotState& state) {

    // Get current state estimate
    Eigen::MatrixXd X = state.getX();
    Eigen::VectorXd Theta = state.getTheta();
    Eigen::MatrixXd P = state.getP();
    int dimX = state.dimX();
    int dimTheta = state.dimTheta();
    int dimP = state.dimP();

    // Map from right invariant to left invariant error temporarily
    if (error_type_==ErrorType::RightInvariant) {
        Eigen::MatrixXd AdjInv = Eigen::MatrixXd::Identity(dimP,dimP);
        AdjInv.block(0,0,dimP-dimTheta,dimP-dimTheta) = Adjoint_SEK3(state_.Xinv()); 
        P = (AdjInv * P * AdjInv.transpose()).eval();
    }

    // Compute Kalman Gain
    Eigen::MatrixXd PHT = P * H.transpose();
    Eigen::MatrixXd S = H * PHT + N;
    Eigen::MatrixXd K = PHT * S.inverse();

    // Compute state correction vector
    Eigen::VectorXd delta = K*Z;
    Eigen::MatrixXd dX = Exp_SEK3(delta.segment(0,delta.rows()-dimTheta));
    Eigen::VectorXd dTheta = delta.segment(delta.rows()-dimTheta, dimTheta);

    // Update state
    Eigen::MatrixXd X_new = X*dX; // Left-Invariant Update
    Eigen::VectorXd Theta_new = Theta + dTheta;

    // Set new state
    state.setX(X_new); 
    state.setTheta(Theta_new);

    // Update Covariance
    Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(dimP,dimP) - K*H;
    Eigen::MatrixXd P_new = IKH * P * IKH.transpose() + K*N*K.transpose(); // Joseph update form

    // Map from left invariant back to right invariant error
    if (error_type_==ErrorType::RightInvariant) {
        Eigen::MatrixXd Adj = Eigen::MatrixXd::Identity(dimP,dimP);
        Adj.block(0,0,dimP-dimTheta,dimP-dimTheta) = Adjoint_SEK3(X_new); 
        P_new = (Adj * P_new * Adj.transpose()).eval(); 
    }

    // Set new covariance
    state.setP(P_new); 
}   

} // end inekf namespace