/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   velocity_impl.cpp
 *  @author Justin Yu
 *  @brief  Implementation for velocity measurement
 *  @date   May 16, 2023
 **/

namespace measurement {
template<typename T>
VelocityMeasurement<T>::VelocityMeasurement()
    : Measurement(VELOCITY), vel_{0, 0, 0} {}

template<typename T>
void VelocityMeasurement<T>::set_velocity(T vx, T vy, T vz) {
  vel_(0) = vx;
  vel_(1) = vy;
  vel_(2) = vz;
}

template<typename T>
Eigen::Matrix<T, 3, 1> VelocityMeasurement<T>::get_velocity() const {
  return vel_;
}

template<typename T>
double VelocityMeasurement<T>::get_vel_mag() const {
  return vel_.norm();
}

template<typename T>
Eigen::Matrix<T, 3, 1> VelocityMeasurement<T>::get_vel_unit_vec() const {
  return vel_.normalized();
}
}    // namespace measurement
