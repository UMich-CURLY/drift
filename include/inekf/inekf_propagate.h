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
#include <algorithm>
#include <boost/circular_buffer.hpp>
#include <iostream>
#include <map>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>
#include "inekf/inekf.h"
#include "inekf/noise_params.h"
#include "inekf/observations.h"
#include "math/lie_group.h"
#include "state/robot_state.h"
#include "utils/sensor_data_t.h"

namespace inekf {

using ContactState = std::pair<int, bool>;

class Propagation {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Propagation(std::shared_ptr<sensor_data_t> sensor_data_buffer, NoiseParams params, ErrorType error_type);


    /// @name Propagation and Correction Methods
    /// @{
    // ======================================================================
    /**
     * @brief Propagates the estimated state mean and covariance forward using
     * inertial measurements. All landmarks positions are assumed to be static.
     * All contacts velocities are assumed to be zero + Gaussian noise.
     * The propagation model currently assumes that the covariance is for the
     * right invariant error.
     *
     * @param[in] imu: 6x1 vector containing stacked angular velocity and linear
     * acceleration measurements
     * @param[in] dt: double indicating how long to integrate the inertial
     * measurements for
     * @return None
     */
    void Propagate(const Eigen::Matrix<double, 6, 1>& imu, double dt, RobotState& state);

    // ======================================================================
    /**
     * @brief
     *
     * @param[in] w:
     * @param[in] a:
     * @param[in] dt:
     * @param[in] state:
     *
     * @return Eigen::MatrixXd:
     */
    Eigen::MatrixXd StateTransitionMatrix(Eigen::Vector3d& w, Eigen::Vector3d& a, double dt, const RobotState& state);

    // ======================================================================
    /**
     * @brief
     *
     * @param[in] phi:
     * @param[in] dt:
     * @param[in] state:
     *
     * @return Eigen::MatrixXd:
     */
    template<int dim = 3>
    Eigen::MatrixXd DiscreteNoiseMatrix(Eigen::MatrixXd& Phi, double dt, const RobotState& state);

    /// @name Setters
    /// @{
    // ======================================================================
    /**
     * @brief Sets the current noise parameters
     *
     * @param[in] params: The noise parameters to be assigned.
     * @return None
     */
    void set_noise_params(NoiseParams params);


    /// @name Getters
    /// @{
    // ======================================================================
    /**
     * @brief Gets the current noise parameters.
     *
     * @param[in] None
     * @return inekf::NoiseParams: The current noise parameters.
     */
    NoiseParams get_noise_params() const;
    /// @}


    /// @}

   private:
    std::shared_ptr<sensor_data_t> sensor_data_buffer_;
    NoiseParams noise_params_;
    ErrorType error_type_;
    const Eigen::Vector3d g_;           // Gravity vector in world frame (z-up)
    Eigen::Vector3d magnetic_field_;    // Magnetic field vector in world frame (z-up)
    bool estimate_bias_ = true;         // Whether to estimate the gyro and accelerometer biases
};
}    // namespace inekf

#endif    // INEKF_INEKF_PROPAGATE_H
