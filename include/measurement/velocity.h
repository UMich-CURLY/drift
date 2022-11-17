/**
 *  @file   imu.h
 *  @author Justin Yu
 *  @brief  Header file for robot velocity estimate
 *  @date   Nov 16, 2022
 **/

#ifndef VELOCITY_H
#define VELOCITY_H

#include "measurement.h"

template<typename T>
class VelocityMeasurement : public Measurement {
 public:
  VelocityMeasurement();    // default constructor

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
   * @return Eigen::Matrix: 3 by 1 cotaining the velocity vector.
   */
  Eigen::Matrix<T, 3, 1> get_velocity() const;

  /**
   * @brief Get the velocity vector magnitude (m/s).
   *
   * @return double: magnitude of velocity vector.
   */
  double get_vel_mag() const;

  /**
   * @brief Get the velocity unit vector.
   *
   * @return Eigen::Matrix: 3 by 1 containing normalized vector.
   */
  Eigen::Matrix<T, 3, 1> get_vel_unit_vec() const;

 private:
  Eigen::Matrix<T, 3, 1> vel_;
};
#include "measurement/impl/velocity_impl.cpp"
#endif
