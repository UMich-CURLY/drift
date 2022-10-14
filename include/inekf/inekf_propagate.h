/* ----------------------------------------------------------------------------
 * Copyright 2022, Ross Hartley, Tingjun Li
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   inekf_correction.h
 *  @author Ross Hartley, Tingjun Li
 *  @brief  Header file for Invariant EKF correction methods 
 *  @date   September 25, 2018
 **/

#ifndef INEKF_INEKF_PROPAGATE_H
#define INEKF_INEKF_PROPAGATE_H 
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include "state/robot_state.h"
#include "inekf/noise_params.h"
#include "inekf/observations.h"
#include "inekf/inekf.h"
#include "math/lie_group.h"

namespace inekf {

using ContactState = std::pair<int, bool>;

class Propagation: public InEKF {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Propagation();
        
    /// @name Propagation and Correction Methods
    /// @{
        // ======================================================================
        /**
         * @brief Propagates the estimated state mean and covariance forward using inertial measurements. 
         * All landmarks positions are assumed to be static.
         * All contacts velocities are assumed to be zero + Gaussian noise.
         * The propagation model currently assumes that the covariance is for the right invariant error.
         * 
         * @param[in] imu: 6x1 vector containing stacked angular velocity and linear acceleration measurements
         * @param[in] dt: double indicating how long to integrate the inertial measurements for
         * @return None
         */
        void Propagate(const Eigen::Matrix<double,6,1>& imu, double dt, RobotState& state);

    private:
        ErrorType error_type_ = ErrorType::LeftInvariant; 
        bool estimate_bias_ = true;  
        RobotState state_;
        NoiseParams noise_params_;
        const Eigen::Vector3d g_; // Gravity vector in world frame (z-up)
        std::map<int,bool> contacts_;
        std::map<int,int> estimated_contact_positions_;
        mapIntVector3d prior_landmarks_;
        std::map<int,int> estimated_landmarks_;
        Eigen::Vector3d magnetic_field_;

        Eigen::MatrixXd StateTransitionMatrix(Eigen::Vector3d& w, Eigen::Vector3d& a, double dt);
        Eigen::MatrixXd DiscreteNoiseMatrix(Eigen::MatrixXd& Phi, double dt);
}; 
} // end inekf namespace

#endif // INEKF_INEKF_PROPAGATE_H
