/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   legged_kinematics.h
 *  @author Justin Yu
 *  @brief  Header file for robot legged kinematics state measurement
 *  @date   Nov 16, 2022
 **/

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "measurement.h"

/**
 * @class LeggedKinematics
 *
 * Derived measurement class containing information
 * about world-frame robot state
 */
class LeggedKinematics : public Measurement {
 public:
  /**
   * @brief Default constructor.
   */
  LeggedKinematics();

  /**
   * @brief Overload constructor.
   */
  LeggedKinematics(const Eigen::Matrix<double, Eigen::Dynamic, 1>& encoders,
                   const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts);

  /**
   * @brief Computes and stores foot position and Jacobian matricies based on
   * encoder values
   */
  virtual void compute_kinematics() = 0;

  /**
   * @brief Set the ground contact state vector.
   *
   * @param[in] contacts Vector of contact states
   */
  void set_contact(const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts);

  /**
   * @brief Set the encoder position state vector.
   *
   * @param[in] encoders Vector of encoder position coefficients (rad)
   */
  void set_joint_state(
      const Eigen::Matrix<double, Eigen::Dynamic, 1>& encoders);

  /**
   * @brief Get the Jacobian matrix.
   *
   * @param[in] leg
   *
   * @return Jacobian Matrix
   */
  Eigen::Matrix<double, 3, Eigen::Dynamic> get_J(int leg) const;

  /**
   * @brief Get the ground contact state vector.
   *
   * @param[in] leg
   *
   * @return bool of contact state
   */
  bool get_contact(int leg) const;

  /**
   * @brief Get the encoder position state vector.
   *
   * @return Vector of encoder position coefficients (rad)
   */
  double get_joint_state(int encoder) const;

  /**
   * @brief Get the world-frame position in Euclidean space.
   *
   * @param[in] leg
   *
   * @return 3-vector of body-to-foot frame position coefficients (m)
   */
  Eigen::Matrix<double, 3, 1> get_kin_pos(int leg) const;

  /**
   * @brief Get the world-frame velocity in Euclidean space.
   *
   * @param[in] leg
   *
   * @return 3-vector of body-to-foot frame velocity (m/s)
   */
  // Eigen::Matrix<double, 3, 1> get_kin_vel(int leg) const;

  /**
   * @brief Get the world-frame effort (force) in Euclidean space.
   *
   * @return 3-vector of Kinematics effort (Newtons).
   */
  // Eigen::Matrix<double, 3, 1> get_kin_effort() const;

 protected:
  Eigen::Matrix<double, 3, Eigen::Dynamic> position_;
  // Eigen::Matrix<double, 3, Eigen::Dynamic> velocity_;
  // Eigen::Matrix<T, 3, Eigen::Dynamic> effort_;
  Eigen::Matrix<double, 3, Eigen::Dynamic> jacobian_;
  Eigen::Matrix<bool, Eigen::Dynamic, 1> contacts_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> encoders_;
};


#endif