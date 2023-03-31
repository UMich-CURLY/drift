/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   base_correction.h
 *  @author Tingjun Li
 *  @brief  Header file for base correction method
 *  @date   November 25, 2022
 **/

#ifndef FILTER_BASE_CORRECTION_H
#define FILTER_BASE_CORRECTION_H

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <vector>

#include <Eigen/Dense>
#include "yaml-cpp/yaml.h"

#include "measurement/measurement.h"
#include "state/robot_state.h"

enum class CorrectionType {
  BASE,
  LEGGED_KINEMATICS,
  VELOCITY,
};

typedef std::queue<std::shared_ptr<Measurement>>
    MeasurementQueue; /**< Queue for storing sensor data. */
typedef std::shared_ptr<MeasurementQueue>
    MeasurementQueuePtr; /**< Pointer to the queue for storing sensor data. */

class Correction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name Constructors
  /// @{
  /**
   * @brief Constructor for the base correction class
   */
  Correction();

  /**
   * @brief Construct a new Correction object with mutex
   *
   * @param[in] sensor_data_buffer_mutex_ptr: a pointer to the mutex of the
   * sensor data buffer
   * @param[in] enable_imu_bias_update: whether to update the IMU bias. Default
   * is false
   */
  Correction(std::shared_ptr<std::mutex> sensor_data_buffer_mutex_ptr,
             bool enable_imu_bias_update = false);
  /// @}

  /// @name Correction
  /// @{
  // ======================================================================
  /**
   * @brief This is a skeleton for the correction method. It should be
   * implemented in the child class.
   *
   * @param[in,out] state: The current state of the robot
   * @return bool: successfully correct state or not (if we do not receive a
   * new message and this method is called it'll return false.)
   */
  virtual bool Correct(RobotState& state);
  /// @}

  /// @name Getters
  /// @{
  // ======================================================================
  /**
   * @brief Get the pointer to the sensor data buffer
   *
   * @return MeasurementQueuePtr: a pointer to the sensor data buffer
   */
  virtual MeasurementQueuePtr get_sensor_data_buffer_ptr();

  /**
   * @brief Get the correction type
   *
   * @return CorrectionType: the correction type
   */
  const CorrectionType get_correction_type() const;

  /**
   * @brief Get the pointer to the mutex of the sensor data buffer
   *
   * @return std::shared_ptr<std::mutex>: a pointer to the mutex of the sensor
   * data buffer
   */
  std::shared_ptr<std::mutex> get_mutex_ptr();
  /// @} End of Getters

 protected:
  const Eigen::Vector3d g_; /**< Gravity vector (m/s^2) in world frame (z-up).
                               Default is 9.81m/s^2 */
  Eigen::Vector3d
      magnetic_field_; /**< Magnetic field vector in world frame (z-up). */
  CorrectionType correction_type_; /**< Correction type. */
  bool enable_imu_bias_update_;    /**< Whether or not to update the IMU bias,
                                      true for update. It stores the value from
                                      config file when the class object is
                                      created.*/
  double t_diff_thres_; /**< Threshold for time difference between two
                           measurements. It stores the value from config file
                           when the class object is created*/
  std::shared_ptr<std::mutex>
      sensor_data_buffer_mutex_ptr_; /**< Mutex for the sensor data buffer. */
};                                   // class Correction

#endif    // end FILTER_BASE_CORRECTION_H
