/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   base_propagation.cpp
 *  @author Tingjun Li
 *  @brief  Source file for base propagation method
 *  @date   November 25, 2022
 **/

#include "drift/filter/base_propagation.h"

using namespace std;

namespace filter {
// Base propagation class
// ======================================================================
// Default constructor
Propagation::Propagation()
    : g_((Eigen::VectorXd(3) << 0, 0, -9.80).finished()),
      magnetic_field_(
          (Eigen::VectorXd(3) << std::cos(1.2049), 0, std::sin(1.2049))
              .finished()) {
  propagation_type_ = PropagationType::BASE;
}

Propagation::Propagation(
    std::shared_ptr<std::mutex> sensor_data_buffer_mutex_ptr)
    : g_((Eigen::VectorXd(3) << 0, 0, -9.80).finished()),
      magnetic_field_(
          (Eigen::VectorXd(3) << std::cos(1.2049), 0, std::sin(1.2049))
              .finished()),
      sensor_data_buffer_mutex_ptr_(sensor_data_buffer_mutex_ptr) {
  propagation_type_ = PropagationType::BASE;
}

// Base method for propagation
bool Propagation::Propagate(RobotState& state) {
  // Just a skeleton, to be implemented in the child class
  return false;
}

// Return measurement queue
MeasurementQueuePtr Propagation::get_sensor_data_buffer_ptr() {
  return nullptr;
}

// Return propagation type
const PropagationType Propagation::get_propagation_type() const {
  return propagation_type_;
}

// Return mutex pointer
std::shared_ptr<std::mutex> Propagation::get_mutex_ptr() {
  return sensor_data_buffer_mutex_ptr_;
}

bool Propagation::set_initial_state(RobotState& state_) {
  // Dummy method
  return true;
}
// Clear the sensor data buffer. Dummy method
void Propagation::clear() {}
}    // namespace filter