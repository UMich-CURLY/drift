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

  /**
   * @brief Set the velocity measurement coefficients (m/s).
   *
   * @param[in] vx: x velocity coefficient.
   * @param[in] vy: y velocity coefficient.
   * @param[in] vz: z velocity coefficient.
   */
  void set_velocity(T vx, T vy, T vz);

  /**
   * @brief Get the velocity measurement coefficients (m/s).
   *
   * @return Velocity: the velocity POD (x, y, z).
   */
  Velocity<T> get_velocity();

  /**
   * @brief Get the velocity vector magnitude (m/s).
   *
   * @return double: magnitude of velocity vector.
   */
  double get_vel_mag();

  /**
   * @brief Get the velocity unit vector.
   *
   * @return Eigen::Matrix: 3 by 1 of doubles containing normalized vector.
   */
  Eigen::Matrix<double, 3, 1> get_vel_unit_vec();

 private:
  Velocity<T> vel_;
  void vel_inv(T vx, T vy, T vz);
};
#include "measurement/impl/velocity_impl.cpp"
#endif
