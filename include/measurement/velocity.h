#ifndef VELOCITY_H
#define VELOCITY_H

#include "measurement.h"

template<typename T>
struct Velocity {
  T x, y, z;
};

template<typename T>
class VelocityMeasurement : public Measurement {
 public:
  VelocityMeasurement();

  void set_velocity(T vx, T vy, T vz);

  Velocity<T> get_velocity();

  double get_vel_mag();

  Eigen::Matrix<double, 3, 1> get_vel_unit_vec();

 private:
  Velocity<T> vel_;
  void vel_inv(T vx, T vy, T vz);
};
#include "measurement/impl/velocity_impl.cpp"
#endif
