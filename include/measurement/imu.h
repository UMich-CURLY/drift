#ifndef IMU_H
#define IMU_H

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
  Eigen::Matrix3d get_rotation_matrix();

  /**
   * @brief Get the imu measurement quaternion coefficients.
   *
   * @return ImuQuaternion: the quaternion POD (w, x, y, z).
   */
  ImuQuaternion<T> get_quaternion();

  /**
   * @brief Get the imu measurement angular velocity coefficients (rad/s).
   *
   * @return ImuAngularVelocity: the angular velocity POD (x, y, z).
   */
  ImuAngularVelocity<T> get_ang_vel();

  /**
   * @brief Get the imu measurement linear acceleration coefficients (m/s^2).
   *
   * @return ImuLinearAcceleration: the linear acceleration POD (x, y, z).
   */
  ImuLinearAcceleration<T> get_lin_acc();

 private:
  Eigen::Matrix3d R_;
  ImuQuaternion<T> quaternion_;
  ImuAngularVelocity<T> angular_velocity_;
  ImuLinearAcceleration<T> linear_acceleration_;
  void set_rotation();
  void quat_inv(T w, T x, T y,
                T z);    // quaternion must be normalized for well defined
                         // Eigen::toRotationMatrix() output
};
#include "measurement/impl/imu_impl.cpp"

#endif
