/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   state_estimator.h
 *  @author Tzu-Yuan Lin, Tingjun Li
 *  @brief  Header file for state estimator class
 *  @date   December 1, 2022
 **/

#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>

#include "filter/base_correction.h"
#include "filter/base_propagation.h"
#include "filter/inekf/correction/legged_kinematics_correction.h"
#include "filter/inekf/correction/velocity_correction.h"
#include "filter/inekf/propagation/imu_propagation.h"
#include "measurement/imu.h"
#include "measurement/legged_kinematics.h"
#include "measurement/velocity.h"
#include "state/robot_state.h"

using aug_map_t = std::map<int, int>;    // Augmented state map {id, aug_idx}
using namespace inekf;

typedef std::queue<std::shared_ptr<RobotState>> RobotStateQueue;
typedef std::shared_ptr<RobotStateQueue> RobotStateQueuePtr;

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
   * @brief Construct a new State Estimator object
   */
  StateEstimator();

  // ======================================================================
  /**
   * @brief Construct a new State Estimator object
   *
   * @param[in] enable_imu_bias_update: True if the filter should update imu
   * bias
   */
  StateEstimator(bool enable_imu_bias_update = false);

  /**
   * @brief Construct a new State Estimator object
   *
   * @param[in] error_type: Error type of the filter
   * @param[in] enable_imu_bias_update: True if the filter should update imu
   * bias
   */
  StateEstimator(ErrorType error_type, bool enable_imu_bias_update = false);
  /// @}

  /// @name Setters
  /// @{
  // ======================================================================
  /**
   * @brief Set the initial state of the robot
   *
   * @param[in] state: Initial state of the robot
   */
  void set_state(const RobotState& state);
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

  // ======================================================================
  /**
   * @brief Get the robot state queue pointer
   *
   * @return RobotStateQueuePtr: Pointer to the robot state queue
   */
  RobotStateQueuePtr get_robot_state_queue_ptr();

  // ======================================================================
  /**
   * @brief Get the robot state queue mutex pointer
   *
   * @return std::shared_ptr<std::mutex>: Pointer to the robot state queue
   * mutex
   */
  std::shared_ptr<std::mutex> get_robot_state_queue_mutex_ptr();
  /// @}


  /// @name Propagation
  /// @{
  // ======================================================================
  /**
   * @brief Declare a propagation method, which uses imu data to propagate
   * the state of the robot
   *
   * @param[in] buffer_ptr: The imu buffer queue temporarily stores the
   * message from the subscriber.
   * @param[in] buffer_mutex_ptr: The imu buffer mutex pointer
   * @param[in] yaml_filepath: The yaml file path for the imu propagation config
   */
  void add_imu_propagation(
      IMUQueuePtr buffer_ptr, std::shared_ptr<std::mutex> buffer_mutex_ptr,
      const std::string& yaml_filepath
      = "config/filter/inekf/propagation/imu_propagation.yaml");
  /// @}

  /// @name Correction
  /// @{
  // ======================================================================
  /**
   * @brief Declare a correction method, which uses leg kinematic data to
   * correct the state of the robot
   *
   * @param[in] buffer_ptr: The kinematic buffer queue temporarily stores the
   * message from the subscriber.
   * @param[in] buffer_mutex_ptr: The kinematic buffer mutex pointer
   * @param[in] yaml_filepath: The yaml file path for the kinematic correction
   */
  void add_legged_kinematics_correction(
      LeggedKinematicsQueuePtr buffer_ptr,
      std::shared_ptr<std::mutex> buffer_mutex_ptr,
      const std::string& yaml_filepath
      = "config/filter/inekf/"
        "correction/mini_cheetah_legged_kinematics_correction.yaml");

  // ======================================================================
  /**
   * @brief Declare a correction method, which uses velocity data to correct
   * the state of the robot
   *
   * @param[in] buffer_ptr: The velocity buffer queue temporarily stores the
   * message from the subscriber.
   * @param[in] buffer_mutex_ptr: The velocity buffer mutex pointer
   * @param[in] yaml_filepath: The yaml file path for the velocity correction
   * config
   */
  void add_velocity_correction(VelocityQueuePtr buffer_ptr,
                               std::shared_ptr<std::mutex> buffer_mutex_ptr,
                               const std::string& yaml_filepath
                               = "config/filter/inekf/"
                                 "correction/velocity_correction.yaml");
  /// @}

  // ======================================================================
  /**
   * @brief Return whether or not the filter is enabled
   *
   * @return const bool: true for enabled, false for disabled
   */
  const bool is_enabled() const;

  // ======================================================================
  /**
   * @brief Enable the filter, change enabled_ to true
   *
   */
  void EnableFilter();

  // ======================================================================
  /**
   * @brief Initialize the IMU bias of the filter
   *
   */
  void InitBias();

  // ======================================================================
  /**
   * @brief Return whether or not the IMU bias is initialized
   *
   * @return const bool: true for initialized, false for not initialized
   */
  const bool BiasInitialized() const;

  // ======================================================================
  /**
   * @brief Initialize the state of the filter using the IMU data
   *
   */
  void InitState();

  // ======================================================================
  /**
   * @brief Run the filter once.
   *
   * Users should first add propagation and correction methods, then call this
   * method. This method will run in a loop, in which the robot state would be
   * propagated and corrected according to the methods added.
   *
   */
  void RunOnce();


  // ======================================================================
  /**
   * @brief Clear and reset the filter
   *
   */
  void clear();

 private:
  RobotState state_;    // state of the robot
  ErrorType error_type_
      = inekf::LeftInvariant;    // Error Type of the InEKF filter
                                 // (LeftInvariant or RightInvariant)
  std::vector<std::shared_ptr<Correction>>
      corrections_;                   // List of correction methods
  std::vector<aug_map_t> aug_maps;    // List of augmented states mapping
  std::shared_ptr<Propagation>
      propagation_;         // Propagation method of the filter
  bool enabled_ = false;    // Boolean value indicating whether the filter is
                            // enabled or not
  bool enable_imu_bias_update_ = false;    // Boolean value indicating whether
                                           // the filter should update imu bias
  bool new_pose_ready_
      = false;    // Boolean value indicating whether a new pose is generated
  RobotStateQueuePtr robot_state_queue_ptr_;    // Pointer to the filter
                                                // estimated robot states queue
  std::shared_ptr<std::mutex>
      robot_state_queue_mutex_ptr_;    // Mutex of the robot
                                       // state queue
};                                     // class StateEstimator
