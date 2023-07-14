/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   imu_impl.cpp
 *  @author Justin Yu
 *  @brief  Implementation for imu measurement
 *  @date   May 16, 2023
 **/

namespace measurement {
template<typename T>
ImuMeasurement<T>::ImuMeasurement()
    : Measurement(IMU),
      quaternion_{1, 0, 0, 0},
      angular_velocity_{0, 0, 0},
      linear_acceleration_{0, 0, 0} {}

template<typename T>
void ImuMeasurement<T>::set_quaternion(T w, T x, T y, T z) {
  validate_quat(w, x, y, z);

  quaternion_.w() = w;
  quaternion_.x() = x;
  quaternion_.y() = y;
  quaternion_.z() = z;
}

template<typename T>
void ImuMeasurement<T>::set_angular_velocity(T x, T y, T z) {
  angular_velocity_.x() = x;
  angular_velocity_.y() = y;
  angular_velocity_.z() = z;
}

template<typename T>
void ImuMeasurement<T>::set_lin_acc(T x, T y, T z) {
  linear_acceleration_.x() = x;
  linear_acceleration_.y() = y;
  linear_acceleration_.z() = z;
}

template<typename T>
Eigen::Matrix<T, 3, 3> ImuMeasurement<T>::get_rotation_matrix() const {
  return quaternion_.toRotationMatrix();
}

template<typename T>
Eigen::Quaternion<T> ImuMeasurement<T>::get_quaternion() const {
  return quaternion_;
}

template<typename T>
Eigen::Matrix<T, 3, 1> ImuMeasurement<T>::get_angular_velocity() const {
  return angular_velocity_;
}

template<typename T>
Eigen::Matrix<T, 3, 1> ImuMeasurement<T>::get_lin_acc() const {
  return linear_acceleration_;
}

// Private Functions

template<typename T>
void ImuMeasurement<T>::validate_quat(T w, T x, T y, T z) {
  double tol = 1e-5;
  T q_mag = w * w + x * x + y * y + z * z;
  if (std::abs(q_mag - 1) >= tol) {
    throw std::invalid_argument("Quaternion arguments must be normalized");
  }
}
}    // namespace measurement