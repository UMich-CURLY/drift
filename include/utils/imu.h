
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
  ImuMeasurement() { type_ = IMU; }

  Eigen::Matrix3d get_rotation() { return R_; }

  void set_rotation() {
    Eigen::Quaternion<double> q(get_orientation().w, get_orientation().x,
                                get_orientation().y, get_orientation().z);
    R_ = q.toRotationMatrix();
  }

  ImuOrientation<T> get_orientation() { return orientation_; }


 private:
  Eigen::Matrix3d R_;
  ImuOrientation<T> orientation_;
  ImuAngularVelocity<T> angular_velocity_;
  ImuLinearAcceleration<T> linear_acceleration_;
};
//}
#endif
