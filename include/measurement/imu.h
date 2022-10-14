
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

  Eigen::Matrix3d get_rotation();

  void set_rotation();

  ImuOrientation<T> get_orientation();


 private:
  Eigen::Matrix3d R_;
  ImuOrientation<T> orientation_;
  ImuAngularVelocity<T> angular_velocity_;
  ImuLinearAcceleration<T> linear_acceleration_;
};
//}

#endif
