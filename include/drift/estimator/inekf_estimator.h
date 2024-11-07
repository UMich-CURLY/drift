/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   inekf_estimator.h
 *  @author Tzu-Yuan Lin, Tingjun Li
 *  @brief  Header file for state estimator class
 *  @date   May 16, 2023
 **/

#ifndef ESTIMATOR_INEKF_ESTIMATOR_H
#define ESTIMATOR_INEKF_ESTIMATOR_H

#include <stdlib.h>
#include <Eigen/Dense>
#include <atomic>
#include <boost/circular_buffer.hpp>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <thread>

#include "drift/filter/base_correction.h"
#include "drift/filter/base_propagation.h"
#include "drift/filter/inekf/correction/legged_kinematics_correction.h"
#include "drift/filter/inekf/correction/position_correction.h"
#include "drift/filter/inekf/correction/velocity_correction.h"
#include "drift/filter/inekf/propagation/imu_propagation.h"
#include "drift/imu_filter/imu_ang_vel_ekf.h"
#include "drift/measurement/angular_velocity.h"
#include "drift/measurement/imu.h"
#include "drift/measurement/legged_kinematics.h"
#include "drift/measurement/navsat.h"
#include "drift/measurement/odom.h"
#include "drift/measurement/velocity.h"
#include "drift/state/robot_state.h"
#include "drift/utils/type_def.h"

using aug_map_t = std::map<int, int>;    // Augmented state map {id, aug_idx}
using namespace filter;
using namespace filter::inekf;
using namespace state;


namespace estimator {
/**
 * @class InekfEstimator
 *
 * @brief class for state estimation using InEKF. This class will be used to
 *estimate the state of the robot using user chosen filter methods.
 **/
class InekfEstimator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name Constructors
  /// @{
  // ======================================================================
  /**
   * @brief Construct a new State Estimator object
   */
  InekfEstimator();

  /**
   * @brief Construct a new State Estimator object according to given settings.
   *
   * @param[in] error_type: Error type of the filter
   * @param[in] config_file: The yaml file path for the filter config
   */
  InekfEstimator(ErrorType error_type, std::string config_file);
  /// @}

  /// @name Destructors
  /// @{
  // ======================================================================
  /**
   * @brief Destroy the State Estimator object
   */
  ~InekfEstimator();

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


  /// @name Propagation adders
  /// @{
  // ======================================================================
  /**
   * @brief Add a propagation method to InekfEstimator object, which uses imu
   * data to propagate the state of the robot. This propagation method will be
   * called in the when the filter is running.
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


  std::pair<IMUQueuePtr, std::shared_ptr<std::mutex>> add_imu_ang_vel_ekf(
      IMUQueuePtr buffer_ptr, std::shared_ptr<std::mutex> buffer_mutex_ptr,
      AngularVelocityQueuePtr ang_vel_buffer_ptr,
      std::shared_ptr<std::mutex> ang_vel_buffer_mutex_ptr,
      const std::string& yaml_filepath);

  /// @}

  /// @name Correction adders
  /// @{
  // ======================================================================
  /**
   * @brief Add a legged kinematics correction method to the InekfEstimator
   * object, which uses leg kinematic data to correct the state of the
   * robot. This correction method will be called in the when the filter is
   * running.
   *
   * @param[in] buffer_ptr: The kinematic buffer queue temporarily stores
   * the message from the subscriber.
   * @param[in] buffer_mutex_ptr: The kinematic buffer mutex pointer
   * @param[in] yaml_filepath: The yaml file path for the kinematic
   * correction
   */
  void add_legged_kinematics_correction(
      LeggedKinQueuePtr buffer_ptr,
      std::shared_ptr<std::mutex> buffer_mutex_ptr,
      const std::string& yaml_filepath
      = "config/filter/inekf/"
        "correction/mini_cheetah_legged_kinematics_correction.yaml");

  // ======================================================================
  /**
   * @brief Add a velocity correction method to the InekfEstimator object, which
   * uses velocity data to correct the state of the robot. This correction
   * method will be called in the when the filter is running.
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
   * @brief Add a position correction method to the InekfEstimator object, which
   * uses position data to correct the state of the robot. This correction
   * method will be called in the when the filter is running.
   *
   * @param[in] buffer_ptr: The position buffer queue temporarily stores the
   * message from the subscriber.
   * @param[in] buffer_mutex_ptr: The position buffer mutex pointer
   * @param[in] yaml_filepath: The yaml file path for the position correction
   * config
   */
  void add_position_correction(OdomQueuePtr buffer_ptr,
                               std::shared_ptr<std::mutex> buffer_mutex_ptr,
                               const std::string& yaml_filepath
                               = "config/filter/inekf/"
                                 "correction/position_correction.yaml");

  /// @name Utility functions
  /// @{
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
  /// @}

  /// @name Activate filter
  /// @{
  // ======================================================================
  /**
   * @brief Run the filter once. In the main function, users should first add
   * propagation and correction methods, then call this method. This method
   * will run the filter once, in which the robot state would be propagated and
   * corrected if new data are received. Users should call this method in a
   * loop. See ROS/example/<TEST>.cpp for examples.
   */
  void RunOnce();
  /// @}

  /// @name Pose logger
  /// @{
  // ======================================================================
  /**
   * @brief Start pose logging thread
   *
   */
  void StartLoggingThread();

  /**
   * @brief Enable the pose logger
   */
  void PoseLoggingThread();
  // @} Pose logger


  // ======================================================================
  /**
   * @brief Clear and reset the filter. Must call InitState() after calling
   * this method to enable the filter again!
   *
   */
  void clear();

  // ======================================================================
  void SlipEstimatorStep();


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
  bool new_pose_ready_
      = false;    // Boolean value indicating whether a new pose is generated
  RobotStateQueuePtr robot_state_queue_ptr_;    // Pointer to the filter
                                                // estimated robot states queue
  std::shared_ptr<std::mutex>
      robot_state_queue_mutex_ptr_;    // Mutex of the robot
                                       // state queue
  bool enable_pose_logger_;            // Boolean value indicating whether
                                       // the filter should log the pose
  std::ofstream outfile_;              // Output file stream for pose logger
  std::ofstream vel_outfile_;          // Output file stream for velocity logger
  std::ofstream vel_body_outfile_;     // Output file stream for velocity in the
                                       // body frame logger
  double pose_log_rate_;               // Pose logger rate
  double last_pub_t_ = 0.0;            // Last published time for pose logger
  std::thread pose_logging_thread_;    // Pose logging thread
  std::mutex
      robot_state_log_queue_mutex_;    // Mutex of the robot state log queue
  RobotStateQueuePtr robot_state_log_queue_ptr_;    // Pointer to the robot
                                                    // state log queue

  double rotation_cov_val_;                  // Covariance value for rotation
  double velocity_cov_val_;                  // Covariance value for velocity
  double position_cov_val_;                  // Covariance value for position
  std::atomic<bool> stop_signal_ = false;    // Stop signal for pose logger
  bool enabled_slip_estimator_ = false;      // Slip estimator enabled
  std::string pose_log_file_ = "";           // Pose log file path
  std::string vel_log_file_ = "";            // Velocity log file path
  int init_count_ = 0;                       // Count for initialization

  std::shared_ptr<imu_filter::ImuAngVelEKF> imu_filter_;
};    // class InekfEstimator
}    // namespace estimator
#endif    // ESTIMATOR_INEKF_ESTIMATOR_H
