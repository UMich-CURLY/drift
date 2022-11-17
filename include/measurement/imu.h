/**
 *  @file   imu.h
 *  @author Justin Yu
 *  @brief  Header file for robot imu estimate measurement
 *  @date   Nov 16, 2022
 **/

#ifndef IMU_H
#define IMU_H

#include "measurement.h"

template<typename T>
class ImuMeasurement : public Measurement {
 public:
  ImuMeasurement();    // default constructor

  /**
   * @brief Set the imu measurement quaternion coefficients.
   *
   * @param[in] w: w real quaternion coefficient.
   * @param[in] x: x imaginary quaternion coefficient.
   * @param[in] y: y imaginary quaternion coefficient.
   * @param[in] z: z imaginary quaternion coefficient.
   */
  void set_quaternion(T w, T x, T y, T z);

  /**
   * @brief Set the imu measurement angular velocity (rad/s).
   *
   * @param[in] x: x axis coefficient.
   * @param[in] y: y axis coefficient.
   * @param[in] z: z axis coefficient.
   */
  void set_ang_vel(T x, T y, T z);

  /**
   * @brief Set the imu measurement linear acceleration (m/s^2).
   *
   * @param[in] x: x axis coefficient.
   * @param[in] y: y axis coefficient.
   * @param[in] z: z axis coefficient.
   */
  void set_lin_acc(T x, T y, T z);

  /**
   * @brief Get the orthonormal 3D rotation matrix for the imu
   * measurement.
   *
   * @return Eigen::Matrix3d: 3x3 Matrix of doubles.
   */
  Eigen::Matrix<T, 3, 3> get_rotation_matrix() const;

  /**
   * @brief Get the imu measurement quaternion coefficients.
   *
   * @return Eigen::Quaternion: the quaternion (w, x, y, z).
   */
  Eigen::Quaternion<T> get_quaternion() const;

  /**
   * @brief Get the imu measurement angular velocity coefficients (rad/s).
   *
   * @return Eigen::Matrix: the angular velocity POD (x, y, z).
   */
  Eigen::Matrix<T, 3, 1> get_ang_vel() const;

  /**
   * @brief Get the imu measurement linear acceleration coefficients (m/s^2).
   *
   * @return Eigen::Matrix: the linear acceleration POD (x, y, z).
   */
  Eigen::Matrix<T, 3, 1> get_lin_acc() const;

 private:
  Eigen::Quaternion<T> quaternion_;
  Eigen::Matrix<T, 3, 1> angular_velocity_;
  Eigen::Matrix<T, 3, 1> linear_acceleration_;

  /**
   * @brief Validate quaternion inputs. Quaternion must be normalized for well
   * defined Eigen::toRotationMatrix() output
   */
  void validate_quat(T w, T x, T y, T z);
};
#include "measurement/impl/imu_impl.cpp"

#endif    // IMU_H
