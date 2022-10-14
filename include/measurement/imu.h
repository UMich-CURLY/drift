#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <iostream>
#include <string>
#include "measurement.h"

template<typename T>
struct ImuQuaternion {
  T w, x, y, z;
};

template<typename T>
struct ImuAngularVelocity {
  T x, y, z;
};

template<typename T>
struct ImuLinearAcceleration {
  T x, y, z;
};

template<typename T>
class ImuMeasurement : public Measurement {
 public:
  ImuMeasurement();
  /**
   * @brief Set the imu measurement quaternion coefficients.
   *
   * @param[in] x: x quaternion coefficient.
   * @param[in] y: y quaternion coefficient.
   * @param[in] z: z quaternion coefficient.
   * @param[in] w: w quaternion coefficient.
   */
  void set_quaternion(T x, T y, T z, T w);

  /**
   * @brief Set the imu measurement angular velocity.
   *
   * @param[in] x: x axis coefficient (rad/s).
   * @param[in] y: y axis coefficient (rad/s).
   * @param[in] z: z axis coefficient (rad/s).
   */
  void set_ang_vel(T x, T y, T z);

  /**
   * @brief Set the imu measurement linear acceleration.
   *
   * @param[in] x: x axis coefficient (m/s^2).
   * @param[in] y: y axis coefficient (m/s^2).
   * @param[in] z: z axis coefficient (m/s^2).
   */
  void set_lin_acc(T x, T y, T z);

  /**
   * @brief Get the orthonormal 3D rotation matrix for the imu
   * measurement.
   *
   * @return Eigen::Matrix3d: 3x3 Matrix of doubles.
   */
  Eigen::Matrix3d get_rotation_matrix();


  /**
   * @brief Get the imu measurement quaternion x coefficient.
   *
   * @return T: the quaternion x coefficient.
   */
  T get_quaternion_x();

  /**
   * @brief Get the imu measurement quaternion y coefficient.
   *
   * @return T: the quaternion y coefficient.
   */
  T get_quaternion_y();

  /**
   * @brief Get the imu measurement quaternion z coefficient.
   *
   * @return T: the quaternion z coefficient.
   */
  T get_quaternion_z();

  /**
   * @brief Get the imu measurement quaternion w coefficient.
   *
   * @return T: the quaternion w coefficient.
   */
  T get_quaternion_w();

  /**
   * @brief Get the imu measurement linear acceleration x coefficient.
   *
   * @return T: the linear acceleration x coefficient.
   */
  T get_lin_acc_x();

  /**
   * @brief Get the imu measurement linear acceleration y coefficient.
   *
   * @return T: the linear acceleration y coefficient.
   */
  T get_lin_acc_y();

  /**
   * @brief Get the imu measurement linear acceleration z coefficient.
   *
   * @return T: the linear acceleration z coefficient.
   */
  T get_lin_acc_z();

  /**
   * @brief Get the imu measurement angular velocity x coefficient.
   *
   * @return T: the angular velocity x coefficient.
   */
  T get_ang_vel_x();

  /**
   * @brief Get the imu measurement angular velocity y coefficient.
   *
   * @return T: the angular velocity y coefficient.
   */
  T get_ang_vel_y();

  /**
   * @brief Get the imu measurement angular velocity z coefficient.
   *
   * @return T: the angular velocity z coefficient.
   */
  T get_ang_vel_z();

 private:
  Eigen::Matrix3d R_;
  ImuQuaternion<T> quaternion_;
  ImuAngularVelocity<T> angular_velocity_;
  ImuLinearAcceleration<T> linear_acceleration_;
  void set_rotation();
  void quat_inv(T x, T y, T z, T w);
};
#include "measurement/impl/imu_impl.cpp"

#endif
