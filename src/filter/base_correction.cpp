/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   base_correction.cpp
 *  @author Tingjun Li
 *  @brief  Source file for base correction method
 *  @date   May 16, 2023
 **/

#include "drift/filter/base_correction.h"

using namespace std;

// Constructor with error type
namespace filter {
Correction::Correction()
    : g_((Eigen::VectorXd(3) << 0, 0, -9.80).finished()),
      magnetic_field_((Eigen::VectorXd(3) << 0, 0, 0).finished()) {
  correction_type_ = CorrectionType::BASE;
}

Correction::Correction(std::shared_ptr<std::mutex> sensor_data_buffer_mutex_ptr)
    : g_((Eigen::VectorXd(3) << 0, 0, -9.80).finished()),
      magnetic_field_((Eigen::VectorXd(3) << 0, 0, 0).finished()),
      sensor_data_buffer_mutex_ptr_(sensor_data_buffer_mutex_ptr) {
  correction_type_ = CorrectionType::BASE;
}


bool Correction::Correct(RobotState& state) {
  // Just a skeleton, to be implemented in the child class
  return false;
}

MeasurementQueuePtr Correction::get_sensor_data_buffer_ptr() { return nullptr; }

const CorrectionType Correction::get_correction_type() const {
  return correction_type_;
}

// Return mutex pointer
std::shared_ptr<std::mutex> Correction::get_mutex_ptr() {
  return sensor_data_buffer_mutex_ptr_;
}

// Set initial velocity to the state. Dummy method
bool Correction::initialize(RobotState& state_) { return true; }

// Clear the sensor data buffer. Dummy method
void Correction::clear() {}
}    // namespace filter