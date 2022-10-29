#ifndef JOINT_STATE_H
#define JOINT_STATE_H

#include "measurement.h"

template<unsigned int JOINT_DIM>
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
  void set_joint_state(const Eigen::Matrix<double, JOINT_DIM, 1>& position,
                       const Eigen::Matrix<double, JOINT_DIM, 1>& velocity,
                       const Eigen::Matrix<double, JOINT_DIM, 1>& effort);

  /**
   * @brief Get the joint position coefficients.
   *
   * @return Eigen::Matrix: vector of joint position coefficients with length
   * JOINT_DIM.
   */
  Eigen::Matrix<double, JOINT_DIM, 1> get_joint_pos();

  /**
   * @brief Get the joint velocity coefficients.
   *
   * @return Eigen::Matrix: vector of joint velocity coefficients with length
   * JOINT_DIM.
   */
  Eigen::Matrix<double, JOINT_DIM, 1> get_joint_vel();

  /**
   * @brief Get the joint effort coefficients.
   *
   * @return Eigen::Matrix: vector of joint effort coefficients with length
   * JOINT_DIM.
   */
  Eigen::Matrix<double, JOINT_DIM, 1> get_joint_effort();


 private:
  Eigen::Matrix<double, JOINT_DIM, 1> joint_position_;
  Eigen::Matrix<double, JOINT_DIM, 1> joint_velocity_;
  Eigen::Matrix<double, JOINT_DIM, 1> joint_effort_;
};
#include "measurement/impl/joint_state_impl.cpp"

#endif
