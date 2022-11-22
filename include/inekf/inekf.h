/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   InEKF.h
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
        ErrorType getErrorType() const;

        // ======================================================================
        /**
         * @brief Gets the current state estimate.
         * 
         * @param[in] None
         * @return RobotState: The current state estimate.
         */
        RobotState getState() const;

        // ======================================================================
        /**
         * @brief Gets the current noise parameters.
         * 
         * @param[in] None
         * @return inekf::NoiseParams: The current noise parameters.
         */
        NoiseParams getNoiseParams() const;

        // ======================================================================
        /**
         * @brief Gets the filter's current contact states.
         * 
         * @param[in] None
         * @return std::map<int,bool>: Map of contact ID and bool that indicates if contact is registed
         */
        std::map<int,bool> getContacts() const;

        // ======================================================================
        /**
         * @brief Get the current estimated contact positions.
         * 
         * @param[in] None
         * @return std::map<int,int> map of contact ID and associated index in the state matrix X
         */
        std::map<int,int> getEstimatedContactPositions() const;

        // ======================================================================
        /**
         * @brief Gets the filter's prior landmarks.
         * 
         * @param[in] None
         * @return mapIntVector3d: map of prior landmark ID and position (as a Eigen::Vector3d)
         */
        mapIntVector3d getPriorLandmarks() const;

        // ======================================================================
        /**
         * @brief Gets the filter's estimated landmarks.
         * 
         * @param[in] None
         * @return std::map<int,int>: map of landmark ID and associated index in the state matrix X
         */
        std::map<int,int> getEstimatedLandmarks() const;

        // ======================================================================
        /**
         * @brief: Gets the filter's set magnetic field.
         * 
         * @param[in] None
         * @return Eigen::Vector3d: magnetic field in world frame
         */
        Eigen::Vector3d getMagneticField() const;
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
        void setState(RobotState state);

        // ======================================================================
        /**
         * @brief Sets the current noise parameters
         * 
         * @param[in] params: The noise parameters to be assigned.
         * @return None
         */
        void setNoiseParams(NoiseParams params);

        // ======================================================================
        /**
         * @brief Sets the filter's current contact state.
         * 
         * @param[in] contacts: A vector of contact ID and indicator pairs. A true indicator means contact is detected.
         * @return None
         */
        void setContacts(std::vector<ContactState> contacts);
        
        // ======================================================================
        /**
         * @brief: Sets the filter's prior landmarks.
         * 
         * @param[in] prior_landmarks: A map of prior landmark IDs and associated position in the world frame.
         * @return None
         */
        void setPriorLandmarks(const mapIntVector3d& prior_landmarks);
        /** TODO: Sets magnetic field for untested magnetometer measurement */
        void setMagneticField(Eigen::Vector3d& true_magnetic_field);
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

        // ======================================================================
        /**
         * @brief Removes a single landmark from the filter's prior landmark set.
         * 
         * @param[in] landmark_id: The ID for the landmark to remove.
         * @return None
         */
        void RemovePriorLandmarks(const int landmark_id);

        // ======================================================================
        /**
         * @brief Removes a set of landmarks from the filter's prior landmark set.
         * 
         * @param[in] landmark_ids: A vector of IDs for the landmarks to remove.
         * @return None
         */
        void RemovePriorLandmarks(const std::vector<int> landmark_ids);

        // ======================================================================
        /**
         * @brief Removes a single landmark from the filter's estimated landmark set.
         * 
         * @param[in] landmark_id: The ID for the landmark to remove.
         * @return None
         */
        void RemoveLandmarks(const int landmark_id);

        // ======================================================================
        /**
         * @brief Removes a set of landmarks from the filter's estimated landmark set.
         * 
         * @param[in] landmark_ids: A vector of IDs for the landmarks to remove.
         * @return None
         */
        void RemoveLandmarks(const std::vector<int> landmark_ids);

        // ======================================================================
        /**
         * @brief Keeps a set of landmarks from the filter's estimated landmark set.
         * 
         * @param[in] landmark_ids: A vector of IDs for the landmarks to keep.
         * @return None
         */
        void KeepLandmarks(const std::vector<int> landmark_ids);
    /// @}


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
        void Propagate(const Eigen::Matrix<double,6,1>& imu, double dt);
        
        // ======================================================================
        /** 
         * @brief Corrects the state estimate using the measured forward kinematics between the IMU and a set of contact frames.
         * If contact is indicated but not included in the state, the state is augmented to include the estimated contact position.
         * If contact is not indicated but is included in the state, the contact position is marginalized out of the state. 
         * This is a right-invariant measurement model. Example usage can be found in @include kinematics.cpp
         * @param[in] measured_kinematics: the measured kinematics containing the contact id, relative pose measurement in the IMU frame, and covariance
         * @return None
         */
        void CorrectKinematics(const vectorKinematics& measured_kinematics); 

        // ======================================================================
        /** 
         * @brief Corrects the state estimate using the measured position between a set of contact frames and the IMU.
         * If the landmark is not included in the state, the state is augmented to include the estimated landmark position. 
         * This is a right-invariant measurement model.
         * 
         * @param[in] measured_landmarks: the measured landmarks containing the contact id, relative position measurement in the IMU frame, and covariance
         * @return None
         */
        void CorrectLandmarks(const vectorLandmarks& measured_landmarks);

        /** TODO: Untested magnetometer measurement*/
        void CorrectMagnetometer(const Eigen::Vector3d& measured_magnetic_field, const Eigen::Matrix3d& covariance);
        /** TODO: Untested GPS measurement*/
        void CorrectPosition(const Eigen::Vector3d& measured_position, const Eigen::Matrix3d& covariance, const Eigen::Vector3d& indices);
        /** TODO: Untested contact position measurement*/
        void CorrectContactPosition(const int id, const Eigen::Vector3d& measured_contact_position, const Eigen::Matrix3d& covariance, const Eigen::Vector3d& indices);
    /// @} 

    /** @example kinematics.cpp
     * Testing
     */

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
         * @param[in] Z: measurement output difference matrix
         * @param[in] H: output matrix
         * @param[in] N: measurement noise matrix
         * @return None
         */
        void CorrectRightInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& N);

        // ======================================================================
        /**
         * @brief Corrects the state using Left Invariant observation model with 
         * given measurement, output and matrices.
         * 
         * @param[in] Z: measurement output difference matrix
         * @param[in] H: measurement error matrix
         * @param[in] N: measurement noise matrix
         * @return None
         */
        void CorrectLeftInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& N);
        // void CorrectFullState(const Observation& obs); // TODO
};

} // end inekf namespace
#endif // end INEKF_INEKF_H