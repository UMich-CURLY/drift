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

#ifndef INEKF_INEKF_CORRECT_H
#define INEKF_INEKF_CORRECT_H 
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
#include "utils.h"

namespace inekf {

using ContactState = std::pair<int, bool>;

class Correction: public InEKF {
    public: 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// @name Constructors
    Correction();

    /// @name Correction method skeletons
    void Correct(RobotState& state); 

};

class KinematicsCorrection: public Correction {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    /// @name Constructors
        KinematicsCorrection();

    /// @name Getters
    /// @{
        // ======================================================================
        /**
         * @brief Gets the filter's current contact states.
         * 
         * @param[in] None
         * @return std::map<int,bool>: Map of contact ID and bool that indicates if contact is registed
         */
        std::map<int,bool> get_contacts() const;

        // ======================================================================
        /**
         * @brief Get the current estimated contact positions.
         * 
         * @param[in] None
         * @return std::map<int,int> map of contact ID and associated index in the state matrix X
         */
        std::map<int,int> get_estimated_contact_positions() const;
    /// @}

    /// @name Setters
    /// @{
        // ======================================================================
        /**
         * @brief Sets the filter's current contact state.
         * 
         * @param[in] contacts: A vector of contact ID and indicator pairs. A true indicator means contact is detected.
         * @return None
         */
        void set_contacts(std::vector<ContactState> contacts);
    /// @}

    /// @name Correction Methods
    /// @{
        // ======================================================================
        /** 
         * @brief Corrects the state estimate using the measured forward kinematics between the IMU and a set of contact frames.
         * If contact is indicated but not included in the state, the state is augmented to include the estimated contact position.
         * If contact is not indicated but is included in the state, the contact position is marginalized out of the state. 
         * This is a right-invariant measurement model. Example usage can be found in @include kinematics.cpp
         * @param[in] measured_kinematics: the measured kinematics containing the contact id, relative pose measurement in the IMU frame, and covariance
         * @return None
         */
        void Correct(const vectorKinematics& measured_kinematics, RobotState& state); 
    /// @}

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

class VelocityCorrection: public Correction {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    /// @name Constructors
        VelocityCorrection();

    /// @name Correction Methods
    /// @{
        // ======================================================================
        /** 
         * @brief Corrects the state estimate using the measured forward kinematics between the IMU and a set of contact frames.
         * If contact is indicated but not included in the state, the state is augmented to include the estimated contact position.
         * If contact is not indicated but is included in the state, the contact position is marginalized out of the state. 
         * This is a right-invariant measurement model. Example usage can be found in @include kinematics.cpp
         * @param[in] measured_kinematics: the measured kinematics containing the contact id, relative pose measurement in the IMU frame, and covariance
         * @return None
         */
        void CorrectVelocity(const Eigen::Vector3d& measured_velocity, const Eigen::Matrix3d& covariance, RobotState& state);
    /// @}

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

#endif // end INEKF_INEKF_CORRECT_H