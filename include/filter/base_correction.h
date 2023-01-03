/* ----------------------------------------------------------------------------
 * Copyright 2022, Tingjun Li, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   base_correction.h
 *  @author Tingjun Li, Ross Hartley
 *  @brief  Header file for Invariant EKF base correction method
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

#include "filter/noise_params.h"
#include "filter/observations.h"
#include "measurement/measurement.h"
#include "state/robot_state.h"

enum class CorrectionType {
  BASE,
  KINEMATICS,
  VELOCITY,
};

typedef std::queue<std::shared_ptr<Measurement>> MeasurementQueue;
typedef std::shared_ptr<MeasurementQueue> MeasurementQueuePtr;

class Correction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name Constructors
  /**
   * @brief Constructor for the base correction class
   */
  Correction();

  /**
   * @brief Construct a new Correction object with mutex
   *
   * @param[in] sensor_data_buffer_mutex_ptr: a pointer to the mutex of the
   * sensor data buffer
   */
  Correction(std::shared_ptr<std::mutex> sensor_data_buffer_mutex_ptr);

  /// @name Correction
  /// @{
  // ======================================================================
  /**
   * @brief This is a skeleton for the correction method. It should be
   * implemented in the child class.
   *
   * @param[in/out] state: The current state of the robot
   * @return bool: successfully correct state or not (if we do not receive a
   * new message and this method is called it'll return false.)
   */
  virtual bool Correct(RobotState& state);
  /// @}

  /// @name Getters
  virtual MeasurementQueuePtr get_sensor_data_buffer_ptr();

  const CorrectionType get_correction_type() const;

  std::shared_ptr<std::mutex> get_mutex_ptr();

 protected:
  const Eigen::Vector3d g_;    // Gravity vector in world frame (z-up)
  Eigen::Vector3d
      magnetic_field_;    // Magnetic field vector in world frame (z-up)
  CorrectionType correction_type_;
  /// TODO: Move this to yaml
  double t_diff_thres_ = 0.3;
  std::shared_ptr<std::mutex> sensor_data_buffer_mutex_ptr_;
};    // class Correction

// #include "../src/filter/base_correction.cpp"
#endif    // end FILTER_BASE_CORRECTION_H
