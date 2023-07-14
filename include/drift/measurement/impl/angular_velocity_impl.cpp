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
AngularVelocityMeasurement<T>::AngularVelocityMeasurement()
    : Measurement(ANGULAR_VELOCITY), ang_vel_{0, 0, 0} {}

template<typename T>
void AngularVelocityMeasurement<T>::set_angular_velocity(T vx, T vy, T vz) {
  ang_vel_(0) = vx;
  ang_vel_(1) = vy;
  ang_vel_(2) = vz;
}

template<typename T>
Eigen::Matrix<T, 3, 1> AngularVelocityMeasurement<T>::get_angular_velocity() const {
  return ang_vel_;
}

template<typename T>
double AngularVelocityMeasurement<T>::get_angular_velocity_magnitude() const {
  return ang_vel_.norm();
}

template<typename T>
Eigen::Matrix<T, 3, 1> AngularVelocityMeasurement<T>::get_ang_vel_unit_vec()
    const {
  return ang_vel_.normalized();
}
}    // namespace measurement
