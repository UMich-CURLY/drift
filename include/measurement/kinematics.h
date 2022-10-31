#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "measurement.h"

template<typename T>
class KinematicsMeasurement : public Measurement {
 public:
  KinematicsMeasurement();

  /**
   * @brief Set the world-frame kinematics state coefficients (m).
   *
   * @param[in] position: vector of kinematics position coefficients.
   * @param[in] velocity: vector of kinematics velocity coefficients.
   * @param[in] effort: vector of kinematics effort coefficients.
   */
  void set_kin_state(const Eigen::Matrix<T, 3, 1>& position,
                     const Eigen::Matrix<T, 3, 1>& velocity,
                     const Eigen::Matrix<T, 3, 1>& effort);

  /**
   * @brief Get the world-frame position coefficients.
   *
   * @return Eigen::Matrix: vector of kinematics position coefficients with
   * length 3.
   */
  Eigen::Matrix<T, 3, 1> get_kin_pos();

  /**
   * @brief Get the world-frame velocity coefficients (m/s).
   *
   * @return Eigen::Matrix: vector of kinematics velocity coefficients with
   * length 3.
   */
  Eigen::Matrix<T, 3, 1> get_kin_vel();

  /**
   * @brief Get the world-frame effort coefficients (Newton).
   *
   * @return Eigen::Matrix: vector of kinematics effort coefficients with length
   * 3.
   */
  Eigen::Matrix<T, 3, 1> get_kin_effort();

 private:
  Eigen::Matrix<T, 3, 1> position_;
  Eigen::Matrix<T, 3, 1> velocity_;
  Eigen::Matrix<T, 3, 1> effort_;
};
#include "measurement/impl/kinematics_impl.cpp"

#endif