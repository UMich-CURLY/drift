#pragma once
#include <stdint.h>
#include <string>
#include "utils/measurement.h"

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
  Eigen::Matrix3d getRotation() { return R_; }

  void setRotation() {
    Eigen::Quaternion<double> q(orientation.w, orientation.x, orientation.y,
                                orientation.z);
    R_ = q.toRotationMatrix();
  }

  // Construct IMU measurement
  ImuMeasurement() { type_ = IMU; }

 private:
  Eigen::Matrix3d R_;
  ImuOrientation<T> orientation_;
  ImuAngularVelocity<T> angular_velocity_;
  ImuLinearAcceleration<T> linear_acceleration_;
};
//}    // namespace cheetah_inekf_lcm