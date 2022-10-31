#ifndef JOINT_STATE_H
#define JOINT_STATE_H

#include "measurement.h"

template<unsigned int JOINT_DIM, typename T>
class JointStateMeasurement : public Measurement {
 public:
  JointStateMeasurement();

  /**
   * @brief Set the joint state coefficients.
   *
   * @param[in] position: vector of joint position coefficients.
   * @param[in] velocity: vector of joint velocity coefficients.
   * @param[in] effort: vector of joint effort coefficients.
   */
  void set_joint_state(const Eigen::Matrix<T, JOINT_DIM, 1>& position,
                       const Eigen::Matrix<T, JOINT_DIM, 1>& velocity,
                       const Eigen::Matrix<T, JOINT_DIM, 1>& effort);

  /**
   * @brief Get the joint-axis position coefficients.
   *
   * @return Eigen::Matrix: vector of joint position coefficients with length
   * JOINT_DIM.
   */
  Eigen::Matrix<T, JOINT_DIM, 1> get_joint_pos();

  /**
   * @brief Get the joint-axis velocity coefficients.
   *
   * @return Eigen::Matrix: vector of joint velocity coefficients with length
   * JOINT_DIM.
   */
  Eigen::Matrix<T, JOINT_DIM, 1> get_joint_vel();

  /**
   * @brief Get the joint-axis effort (torque) coefficients (Newton-meters).
   *
   * @return Eigen::Matrix: vector of joint effort coefficients with length
   * JOINT_DIM.
   */
  Eigen::Matrix<T, JOINT_DIM, 1> get_joint_effort();


 private:
  Eigen::Matrix<T, JOINT_DIM, 1> joint_position_;
  Eigen::Matrix<T, JOINT_DIM, 1> joint_velocity_;
  Eigen::Matrix<T, JOINT_DIM, 1> joint_effort_;
};
#include "measurement/impl/joint_state_impl.cpp"

#endif
