#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <string>
#include "measurement.h"

// namespace cheetah_inekf_lcm {

template<typename T>
struct ImuOrientation {
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
   * @brief Get the 3D rotation matrix for the imu
   * measurement.
   *
   * @return Eigen::Matrix3d: 3x3 Matrix of doubles.
   */
  Eigen::Matrix3d get_rotation_matrix();


  /**
   * @brief Get the imu measurement quaternion x coefficient.
   *
   * @return ImuOrientation<T>: the quaternion x coefficient.
   */
  T get_quaternion_x();

  /**
   * @brief Get the imu measurement quaternion y coefficient.
   *
   * @return ImuOrientation<T>: the quaternion y coefficient.
   */
  T get_quaternion_y();

  /**
   * @brief Get the imu measurement quaternion z coefficient.
   *
   * @return ImuOrientation<T>: the quaternion z coefficient.
   */
  T get_quaternion_z();

  /**
   * @brief Get the imu measurement quaternion w coefficient.
   *
   * @return ImuOrientation<T>: the quaternion w coefficient.
   */
  T get_quaternion_w();

  /**
   * @brief Set the imu measurement quaternion coefficients.
   *
   * @param[in] x: x quaternion coefficient.
   * @param[in] y: y quaternion coefficient.
   * @param[in] z: z quaternion coefficient.
   * @param[in] w: w quaternion coefficient.
   */
  void set_quaternion(T x, T y, T z, T w);


 private:
  Eigen::Matrix3d R_;
  ImuOrientation<T> orientation_;
  ImuAngularVelocity<T> angular_velocity_;
  ImuLinearAcceleration<T> linear_acceleration_;
  void set_rotation();
};
//}
#include "measurement/impl/imu_impl.cpp"

#endif
