/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   legged_kinematics.h
 *  @author Justin Yu
 *  @brief  Header file for legged robot kinematics state measurement
 *  @date   May 16, 2023
 **/

#ifndef MEASUREMENT_LEG_KINEMATICS_H
#define MEASUREMENT_LEG_KINEMATICS_H

#include "measurement.h"

namespace measurement {
/**
 * @class LeggedKinematicsMeasurement
 *
 * @brief Derived measurement class containing information
 * about body-to-foot kinematics
 */
class LeggedKinematicsMeasurement : public Measurement {
 public:
  /// @name Constructors
  /**
   * @brief Default constructor, will generate an empty measurement.
   */
  LeggedKinematicsMeasurement();

  /**
   * @brief Overload constructor.
   * @param[in] encoders Vector of encoder position coefficients (rad)
   * @param[in] d_encoders Vector of encoder velocity coefficients (rad/s)
   * @param[in] contacts Vector of contact states
   */
  LeggedKinematicsMeasurement(
      const Eigen::Matrix<double, Eigen::Dynamic, 1>& encoders,
      const Eigen::Matrix<double, Eigen::Dynamic, 1>& d_encoders,
      const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts);
  /// @}

  /// @name Util functions
  /// @{
  /**
   * @brief Computes and stores foot position and Jacobian matricies based on
   * encoder values. This method will be overloaded by the derived class.
   */
  virtual void ComputeKinematics() = 0;
  /// @}

  /// @name Getter
  /// @{
  /**
   * @brief Returns the number of actuated robot legs. This method will be
   * overloaded by the derived class.
   */
  virtual int get_num_legs() = 0;

  /**
   * @brief Returns the initial velocity of the robot. This method will be
   * overloaded by the derived class.
   *
   * @param[in] w Angular velocity of the robot
   * @return Initial velocity of the robot
   */
  virtual const Eigen::Vector3d get_init_velocity(const Eigen::Vector3d& w);

  /**
   * @brief Get the Jacobian matrix.
   *
   * @param[in] leg Leg id
   *
   * @return Jacobian Matrix
   */
  Eigen::Matrix<double, 3, Eigen::Dynamic> get_J(int leg) const;

  /**
   * @brief Get the ground contact state vector.
   *
   * @param[in] leg Leg id
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
   * @param[in] leg Leg id
   *
   * @return 3D-vector of body-to-foot frame position coefficients (m)
   */
  Eigen::Matrix<double, 3, 1> get_kin_pos(int leg) const;
  /// @}

  /// @name Setter
  /// @{
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
   * @brief Set the encoder velocity state vector.
   *
   * @param[in] encoders Vector of encoder position coefficients (rad)
   */
  void set_joint_state_velocity(
      const Eigen::Matrix<double, Eigen::Dynamic, 1>& d_encoders);
  /// @}

 protected:
  Eigen::Matrix<double, 3, Eigen::Dynamic>
      positions_; /**< Position of each feet in world frame. Stored
                 in a [3 x n] matrix while 3 represents
                 (x, y, z) and n represents number of legs*/

  Eigen::Matrix<double, 3, Eigen::Dynamic>
      jacobians_; /**< Jacobian matrices of each feet in world frame. */

  Eigen::Matrix<bool, Eigen::Dynamic, 1> contacts_;   /**< Contact state of each
      feet, "true" for having contact, "false" for not having contact*/
  Eigen::Matrix<double, Eigen::Dynamic, 1> encoders_; /**< Encoder position
                                                        state vector. */
  Eigen::Matrix<double, Eigen::Dynamic, 1> d_encoders_; /**< Encoder velocity
                                                          state vector. */
};
}    // namespace measurement

#endif    // MEASUREMENT_LEG_KINEMATICS_H