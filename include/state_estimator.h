/* ----------------------------------------------------------------------------
 * Copyright 2022, Tingjun Li
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   state_estimator.h
 *  @author Tingjun Li
 *  @brief  Header file for state estimator class
 *  @date   December 1, 2022
 **/

#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>
#include <iostream>
#include <map>
#include <memory>

#include "filter/base_correction.h"
#include "filter/base_propagation.h"
#include "filter/inekf/correction/kinematics_correction.h"
#include "filter/inekf/correction/velocity_correction.h"
#include "filter/inekf/propagation/imu_propagation.h"
#include "state/robot_state.h"

using aug_map_t = std::map<int, int>;    // Augmented state map {id, aug_idx}
using namespace inekf;

/**
 * @class StateEstimator
 *
 * A class for state estimation. This class will be used to estimate the state
 * of the robot using user chosen filter methods.
 **/
class StateEstimator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name Constructors
  /// @{
  // ======================================================================
  /**
   * @brief
   *
   * @param[in] params: Noise parameters
   * @param[in] error_type: Error type of the filter
   */
  StateEstimator(NoiseParams params, ErrorType error_type);
  /// @}

  /// @name Setters
  /// @{
  // ======================================================================
  /**
   * @brief Set the initial state of the robot
   *
   * @param[in] state: Initial state of the robot
   */
  void set_state(RobotState& state);
  /// @}

  /// @name Getters
  /// @{
  // ======================================================================
  /**
   * @brief Get the state of the robot
   *
   * @return RobotState: State of the robot
   */
  const RobotState get_state() const;
  /// @}


  /// @name Propagation
  /// @{
  // ======================================================================
  /**
   * @brief Declare a propagation method, which uses imu data to propagate the
   * state of the robot
   *
   * @param[in] buffer_ptr: The imu buffer queue temporarily stores the message
   * from the subscriber.
   */
  template<typename imu_q_t>
  void add_imu_propagation(std::shared_ptr<imu_q_t> buffer_ptr,
                           const bool estimate_bias);
  /// @}

  /// @name Correction
  /// @{
  // ======================================================================
  /**
   * @brief Declare a correction method, which uses landmark data to correct
   * the state of the robot
   *
   * @param[in] buffer_ptr: The landmark buffer queue temporarily stores the
   * message from the subscriber.
   */
  template<typename landmark_q_t>
  void add_landmark_correction(std::shared_ptr<landmark_q_t> buffer_ptr);

  // ======================================================================
  /**
   * @brief Declare a correction method, which uses kinematic data to correct
   * the state of the robot
   *
   * @param[in] buffer_ptr: The kinematic buffer queue temporarily stores the
   * message from the subscriber.
   */
  template<typename kinematic_q_t>
  void add_kinematics_correction(std::shared_ptr<kinematic_q_t> buffer_ptr);

  // ======================================================================
  /**
   * @brief Declare a correction method, which uses velocity data to correct
   * the state of the robot
   *
   * @param[in] buffer_ptr: The velocity buffer queue temporarily stores the
   * message from the subscriber.
   */
  template<typename velocity_q_t>
  void add_velocity_correction(std::shared_ptr<velocity_q_t> buffer_ptr,
                               const Eigen::Matrix3d& covariance);
  /// @}

  // ======================================================================
  /**
   * @brief Run the filter.
   *
   * Users should first add propagation and correction methods, then call this
   * method. This method will run in a loop, in which the robot state would be
   * propagated and corrected according to the methods added.
   *
   * @param[in] dt: Time step
   */
  void run(double dt);

 private:
  RobotState state_;
  NoiseParams params_;
  ErrorType error_type_;
  // std::shared_ptr<Correction> correction_;
  std::vector<std::shared_ptr<Correction>> corrections_;
  std::vector<aug_map_t> aug_maps;
  std::shared_ptr<Propagation> propagation_;
};    // class StateEstimator
