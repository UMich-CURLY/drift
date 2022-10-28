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
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include "state/robot_state.h"
#include "inekf/noise_params.h"
#include "inekf/observations.h"
#include "math/lie_group.h"
#include "utils.h"

namespace inekf {

enum ErrorType {LeftInvariant, RightInvariant};

using ContactState = std::pair<int,bool>;

class InEKF {
    
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// @name Constructors
    /// @{
        // ======================================================================
        /**
         * @brief Constructor. Initializes the filter with default state (identity 
         * rotation, zero velocity, zero position) and noise parameters.
         * 
         * @param[in] None
        */
        InEKF();

        // ======================================================================
        /**
         * @brief Constructor. Initialize filter with noise parameters and default state
         * (identity rotation, zero velocity, zero position).
         *
         * @param[in] params: The noise parameters to be assigned.
         */
        InEKF(NoiseParams params);

        // ======================================================================
        /**
         * @brief Constructor. Initialize filter with given state.
         * 
         * @param[in] state: The state to be assigned.
         */
        InEKF(RobotState state);

        // ======================================================================
        /**
         * @brief Constructor. Initialize filter with state and noise parameters.
         * 
         * @param[in] state: The state to be assigned.
         * @param[in] params: The noise parameters to be assigned.
         */        
        InEKF(RobotState state, NoiseParams params);

        // ======================================================================
        /**
         * @brief Constructor. Initialize filter with state, noise, and error type.
         * @param[in] state: The state to be assigned.
         * @param[in] params: The noise parameters to be assigned.
         * @param[in] error_type: The type of invariant error to be used (affects covariance).
         */       
        InEKF(RobotState state, NoiseParams params, ErrorType error_type);
    /// @}

    
    /// @name Getters
    /// @{
        // ======================================================================
        /**
         * @brief Gets the current error type.
         * 
         * @param[in] None
         * @return inekf::ErrorType: The current error type.
         */
        ErrorType get_error_type() const;

        // ======================================================================
        /**
         * @brief Gets the current state estimate.
         * 
         * @param[in] None
         * @return RobotState: The current state estimate.
         */
        RobotState get_state() const;

        // ======================================================================
        /**
         * @brief Gets the current noise parameters.
         * 
         * @param[in] None
         * @return inekf::NoiseParams: The current noise parameters.
         */
        NoiseParams get_noise_params() const;

    /// @}


    /// @name Setters
    /// @{
        // ======================================================================
        /**
         * @brief Sets the current state estimate
         * 
         * @param[in] state: The state to be assigned.
         * @return None
         */
        void set_state(RobotState state);

        // ======================================================================
        /**
         * @brief Sets the current noise parameters
         * 
         * @param[in] params: The noise parameters to be assigned.
         * @return None
         */
        void set_noise_params(NoiseParams params);

        // ======================================================================
        /** TODO: Sets magnetic field for untested magnetometer measurement */
        void set_magnetic_field(Eigen::Vector3d& true_magnetic_field);
    /// @}


    /// @name Basic Utilities
    /// @{
        // ======================================================================
        /**
         * @brief Resets the filter
         * Initializes state matrix to identity, removes all augmented states, and assigns default noise parameters.
         * 
         * @param[in] None
         * @return None
         */
        void clear();
    /// @}


    // Corrects state using invariant observation models
        // ======================================================================
        /**
         * @brief Corrects the state using Right Invariant observation model with
         * a given observation.
         * 
         * @param[in] obs: an observation input
         * @return None
         */
        void CorrectRightInvariant(const Observation& obs);

        // ======================================================================
        /**
         * @brief Corrects the state using Left Invariant observation model with
         * a given observation.
         * 
         * @param[in] obs: an observation input
         * @return None
         */
        void CorrectLeftInvariant(const Observation& obs);

        // ======================================================================
        /**
         * @brief Corrects the state using Right Invariant observation model with 
         * given measurement, output and matrices.
         * 
         * @param[in] Z: innovation matrix
         * @param[in] H: measurement error matrix
         * @param[in] N: measurement noise matrix
         * @return None
         */
        void CorrectRightInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& N);

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
        void CorrectRightInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& N, RobotState& state);

        // ======================================================================
        /**
         * @brief Corrects the state using Left Invariant observation model with 
         * given measurement, output and matrices.
         * 
         * @param[in] Z: innovation matrix
         * @param[in] H: measurement error matrix
         * @param[in] N: measurement noise matrix
         * @return None
         */
        void CorrectLeftInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& N);

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
        void CorrectLeftInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& N, RobotState& state);
        // void CorrectFullState(const Observation& obs); // TODO

    /** @example kinematics.cpp
     * Testing
     */

    protected:
        ErrorType error_type_ = ErrorType::LeftInvariant; 
        bool estimate_bias_ = true;  
        RobotState state_;
        NoiseParams noise_params_;
        const Eigen::Vector3d g_; // Gravity vector in world frame (z-up)
        Eigen::Vector3d magnetic_field_; // Magnetic field vector in world frame (z-up)

        Eigen::MatrixXd StateTransitionMatrix(Eigen::Vector3d& w, Eigen::Vector3d& a, double dt);
        Eigen::MatrixXd DiscreteNoiseMatrix(Eigen::MatrixXd& Phi, double dt, const std::map<int,int>& augmented_states={});
};

} // end inekf namespace
#endif // end INEKF_INEKF_H