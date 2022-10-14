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
   *
   * @return Eigen::Matrix3d: 3x3 Matrix.
   */
  Eigen::Matrix3d get_rotation();

  /**
   * @brief Translate quaternion values to an equivalent rotation matrix.
   *
   * @return none
   */
  void set_rotation();

  /**
   * @brief Get the imu measurement quaternion w coefficient.
   *
   * @return ImuOrientation<T>: the quaternion w coefficient.
   */
  ImuOrientation<T> orientation_w();

  /**
   * @brief Get the imu measurement quaternion x coefficient.
   *
   * @return ImuOrientation<T>: the quaternion x coefficient.
   */
  ImuOrientation<T> orientation_x();

  /**
   * @brief Get the imu measurement quaternion y coefficient.
   *
   * @return ImuOrientation<T>: the quaternion y coefficient.
   */
  ImuOrientation<T> orientation_y();

  /**
   * @brief Get the imu measurement quaternion z coefficient.
   *
   * @return ImuOrientation<T>: the quaternion z coefficient.
   */
  ImuOrientation<T> orientation_z();

 private:
  Eigen::Matrix3d R_;
  ImuOrientation<T> orientation_;
  ImuAngularVelocity<T> angular_velocity_;
  ImuLinearAcceleration<T> linear_acceleration_;
};
//}
#include "measurement/impl/imu_impl.cpp"

#endif
