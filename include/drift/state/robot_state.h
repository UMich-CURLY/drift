/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   robot_state.h
 *  @author Wenzhe Tong, Tingjun Li
 *  @brief  Header file for robot state
 *  @date   May 16, 2023
 **/

#ifndef STATE_ROBOT_STATE_H
#define STATE_ROBOT_STATE_H

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>

#include "drift/math/lie_group.h"
#include "drift/math/se_k_3.h"


enum StateType { WorldCentric, BodyCentric };

using namespace math;

namespace state {
/**
 * @class RobotState
 *
 * @brief state class containing its pose and augmented states,
 * biases, and covariances.
 */
class RobotState {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// @name Constructors
  /// @{
  /**
   * @brief Default RobotState constructor.
   */
  RobotState();

  /**
   * @brief Construct a new Robot State object with state matrix X.
   *
   * @param[in] X: matrix of SE or SEk group represents for robot state.
   */
  RobotState(const Eigen::MatrixXd& X);

  /**
   * @brief Construct a new Robot State object with state matrix X, and bias
   * matrix Theta.
   *
   * @param[in] X: matrix of SE or SEk group represents for robot state.
   * @param[in] Theta: matrix of bias respect to X.
   */
  RobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta);

  /**
   * @brief Construct a new Robot State object with state matrix X, bias matrix
   * Theta, and covariance matrix P.
   *
   * @param[in] X: matrix of SE or SEk group represents for robot state.
   * @param[in] Theta: matrix of bias respect to X.
   * @param[in] P: matrix of covariance respect to X.
   */
  RobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta,
             const Eigen::MatrixXd& P);

  /**
   * @brief Construct a new Robot State object with state matrix X.
   *
   * @param[in] X: instance of SEK3 class.
   */
  RobotState(SEK3& X);

  /**
   * @brief Construct a new Robot State object with state matrix X, and bias
   * matrix Theta
   *
   * @param[in] X: instance of SEK3 class.
   * @param[in] Theta: matrix of bias respect to X.
   */
  RobotState(SEK3& X, const Eigen::VectorXd& Theta);

  /**
   * @brief Construct a new Robot State object with state matrix X, bias matrix
   * Theta and covariance matrix P.
   *
   * @param[in] X: instance of SEK3 class.
   * @param[in] Theta: matrix of bias respect to X.
   * @param[in] P: matrix of covariance respect to X.
   */
  RobotState(SEK3& X, const Eigen::VectorXd& Theta, const Eigen::MatrixXd& P);
  /// @}

  /// @name Getters
  /// @{
  /**
   * @brief Get state matrix X.
   *
   * @return const Eigen::MatrixXd X: represents for state matrix.
   */
  const Eigen::MatrixXd get_X() const;

  /**
   * @brief Get the bias matrix Theta.
   *
   * @return const Eigen::VectorXd Theta: represents for bias matrix.
   */
  const Eigen::VectorXd get_theta() const;

  /**
   * @brief Get the covariance matrix P.
   *
   * @return const Eigen::MatrixXd P: represents for covariance matrix.
   */
  const Eigen::MatrixXd get_P() const;

  /**
   * @brief Get the Rotation matrix R.
   *
   * @return const Eigen::MatrixXd R: represents for rotation matrix.
   */
  const Eigen::Matrix3d get_rotation() const;

  /**
   * @brief Get the Velocity matrix V.
   *
   * @return const Eigen::Vector3d V: represents for velocity matrix.
   */
  const Eigen::Vector3d get_velocity() const;

  /**
   * @brief Get the Position matrix p.
   *
   * @return const Eigen::Vector3d p: represents for position matrix.
   */
  const Eigen::Vector3d get_position() const;

  /**
   * @brief Get the gyroscope bias vector b.
   *
   * @return const Eigen::Vector3d b: represents for gyroscope bias vector.
   */
  const Eigen::Vector3d get_gyroscope_bias() const;

  /**
   * @brief Get the a 3 by 1 vector from the state
   *
   * @return const Eigen::Vector3d
   */
  const Eigen::Vector3d get_vector(int index) const;

  /**
   * @brief Get the accelerometer bias matrix b.
   *
   * @return const Eigen::Vector3d b: represents for accelerometer bias
   * matrix.
   */
  const Eigen::Vector3d get_accelerometer_bias() const;

  /**
   * @brief Get the rotation covariance matrix.
   *
   * @return const Eigen::Matrix3d: represents for rotation covariance matrix.
   */
  const Eigen::Matrix3d get_rotation_covariance() const;

  /**
   * @brief Get the velocity covariance matrix.
   *
   * @return const Eigen::Matrix3d: represents for velocity covariance matrix.
   */
  const Eigen::Matrix3d get_velocity_covariance() const;

  /**
   * @brief Get the position covariance matrix.
   *
   * @return const Eigen::Matrix3d: represents for position covariance matrix.
   */
  const Eigen::Matrix3d get_position_covariance() const;

  /**
   * @brief Get the gyroscope bias covariance matrix.
   *
   * @return const Eigen::Matrix3d: represents for gyroscope bias covariance
   * matrix.
   */
  const Eigen::Matrix3d get_gyroscope_bias_covariance() const;

  /**
   * @brief Get the accelerometer bias covariance matrix.
   *
   * @return const Eigen::Matrix3d: represents for accelerometer bias
   * covariance matrix.
   */
  const Eigen::Matrix3d get_accelerometer_bias_covariance() const;

  /**
   * @brief Get timestamp of the latest modified state.
   *
   * @return double t: timestamp of the latest modified state.
   */
  const double get_time() const;

  /**
   * @brief Get timestamp of the state propagation.
   *
   * @return double t: timestamp of the state propagation.
   */
  const double get_propagate_time() const;

  /**
   * @brief Get the augmented state vector giving matrix_idx in the state.
   *
   * @param[in] matrix_idx: index of the augmented state in the state.

  */
  const Eigen::Vector3d get_aug_state(int matrix_idx);

  /**
   * @brief get the dimension of the state matrix X.
   *
   * @return const int: dimension of the state matrix X.
   */
  const int dimX() const;

  /**
   * @brief get the dimension of the augmented state matrix, X.
   *
   * @return const int: dimension of the augmented state matrix X.
   */
  const int dimTheta() const;

  /**
   * @brief get the dimension of the covariance matrix, P.
   *
   * @return const int: dimension of the augmented covariance matrix P.
   */
  const int dimP() const;

  /**
   * @brief get the dimension of the continuous noise covariance, Qc.
   *
   * @return const int: dimension of the continuous noise covariance, Qc.
   */
  const int dimQc() const;

  /**
   * @brief get the the state type from: {WorldCentric, BodyCentric}
   *
   * @return const StateType: dimension of the augmented covariance matrix P.
   */
  const StateType get_state_type() const;

  /**
   * @brief Get the WorldCentric state matrix X.
   *
   * @return const Eigen::MatrixXd: state matrix X in world frame.
   */
  const Eigen::MatrixXd get_world_X() const;

  /**
   * @brief Get the WorldCentric Rotation matrix R.
   *
   * @return const Eigen::Matrix3d: state matrix R in the world frame.
   */
  const Eigen::Matrix3d get_world_rotation() const;

  /**
   * @brief Get the WorldCentric Velocity vector v.
   *
   * @return const Eigen::Vector3d: velocity vector v in the world frame.
   */
  const Eigen::Vector3d get_world_velocity() const;

  /**
   * @brief Get the WorldCentric Position vector p.
   *
   * @return const Eigen::Vector3d: position vector p in the world frame.
   */
  const Eigen::Vector3d get_world_position() const;

  /**
   * @brief Get the BodyCentric state matrix X.
   *
   * @return const Eigen::MatrixXd: state matrix X in body frame.
   */
  const Eigen::MatrixXd get_body_X() const;

  /**
   * @brief Get the BodyCentric Rotation matrix R.
   *
   * @return const Eigen::Matrix3d: state matrix R in the body frame.
   */
  const Eigen::Matrix3d get_body_rotation() const;

  /**
   * @brief Get the BodyCentric Velocity vector v.
   *
   * @return const Eigen::Vector3d: velocity vector v in the body frame.
   */
  const Eigen::Vector3d get_body_velocity() const;

  /**
   * @brief Get the BodyCentric Position vector p.
   *
   * @return const Eigen::Vector3d: position vector p in the body frame.
   */
  const Eigen::Vector3d get_body_position() const;

  /**
   * @brief Get the continuous noise covariance matrix
   *
   * @return Eigen::MatrixXd: Continuous noise covariance matrix
   */
  const Eigen::MatrixXd get_continuous_noise_covariance() const;

  /**
   * @brief Get the Gyroscope Bias vector b_aug according to matrix_idx.
   *
   * @param[in] matrix_idx: the idx in the state matrix where the aug state is
   * located
   * @return const Eigen::Vector3d: gyroscope bias vector baug.
   */
  const Eigen::Vector3d get_aug_bias(int matrix_idx);

  /**
   * @brief Get the aug cov matrix according to matrix column idx (matrix_idx).
   *
   * @param[in] matrix_idx: the idx in the state matrix where the aug state is
   * located
   * @return const Eigen::Matrix3d: augmented covariance matrix.
   */
  const Eigen::Matrix3d get_aug_cov(int matrix_idx);

  /**
   * @brief compute the inverse of the state matrix X_.
   *
   * @return const Eigen::MatrixXd: inverse of the state matrix X_.
   */
  const Eigen::MatrixXd get_Xinv() const;

  /**
   * @brief Get the BodyCentric Angular Velocity vector w.
   *
   * @return const Eigen::Vector3d: angular velocity vector w in the body
   * frame.
   */
  const Eigen::Vector3d get_body_angular_velocity() const;

  /**
   * @brief Get the boolean value enable_imu_bias_update_
   *
   * @return const bool: boolean value enable_imu_bias_update_
   */
  const bool get_enable_imu_bias_update() const;

  /**
   * @brief Get the slip flag object
   *
   * @return const int slip_flag
   */
  const int get_slip_flag() const;

  /// @} End of Getters

  /// @name Setters
  /// @{
  /**
   * @brief Set the BodyCentric Angular Velocity vector w.
   *
   * @param[in] const Eigen::Vector3d: angular velocity vector w in the body
   * frame.
   */
  void set_body_angular_velocity(const Eigen::Vector3d& w);

  /**
   * @brief Add the augmented state
   *
   * @param[in] aug: Augmented state to be added to the index mapping.
   * @param[in] P_aug: Covariance of the state that is augmented.
   * @param[in] noise_cov: Covariance of noise.
   * @param[in] col_id_ptr: the column id passed by correction mapping.
   */
  int add_aug_state(const Eigen::Vector3d& aug, const Eigen::MatrixXd& P_aug,
                    const Eigen::Matrix3d& noise_cov,
                    std::shared_ptr<int> col_id_ptr);

  /**
   * @brief Set the augmented to the certain position (matrix_idx-th) in the
   * state vector.
   *
   * @param[in] matrix_idx: index of the augmented state in the state.
   * @param[in] aug: augmented state to be set.
   */
  void set_aug_state(int matrix_idx, const Eigen::Vector3d& aug);

  /**
   * @brief Delete certain augmented state (matrix_idx-th) from the state
   * matrix.
   *
   * @param[in] matrix_idx: index of the augmented state in the state.
   */
  void del_aug_state(int matrix_idx);

  /**
   * @brief Delete certain augmented state (matrix_idx-th) from the augmented
   * noise covariance matrix.
   *
   * @param[in] matrix_idx: index of the augmented state in the state.
   */
  void del_aug_noise_cov(int matrix_idx);

  /**
   * @brief Set the RobotState whole matrix.
   *
   * @param[in]: X: state matrix X.
   */
  void set_X(const Eigen::MatrixXd& X);

  /**
   * @brief Set the RobotState Bias matrix P.
   *
   * @param[in] P: bias matrix P.
   */
  void set_P(const Eigen::MatrixXd& P);

  /**
   * @brief Set the RobotState covariance matrix Theta.
   *
   * @param[in] Theta: covariance matrix Theta.
   */
  void set_theta(const Eigen::VectorXd& Theta);

  /**
   * @brief Set the Rotation matrix R in RobotState.
   *
   * @param[in] R: 3x3 rotation matrix R.
   */
  void set_rotation(const Eigen::Matrix3d& R);

  /**
   * @brief Set the Velocity vector v in RoboState.
   *
   * @param[in] v: 3x1 velocity vector v.
   */
  void set_velocity(const Eigen::Vector3d& v);

  /**
   * @brief Set the Position vector p in RoboState.
   *
   * @param[in] p: 3x1 position vector p.
   */
  void set_position(const Eigen::Vector3d& p);

  /**
   * @brief Set the Gyroscope Bias to bias matrix P.
   *
   * @param[in] bg: 3x1 gyroscope bias vector bg.
   */
  void set_gyroscope_bias(const Eigen::Vector3d& bg);

  /**
   * @brief Set the Accelerometer Bias to bias matrix P.
   *
   * @param[in] ba: 3x1 accelerometer bias vector ba.
   */
  void set_accelerometer_bias(const Eigen::Vector3d& ba);

  /**
   * @brief Set timestamp to the state
   *
   * @param[in] t: timestamp of the state
   */
  void set_time(const double t);

  /**
   * @brief Set timestamp of the state propagation.
   *
   * @param[in] t: timestamp of the state propagation.
   */
  void set_propagate_time(const double t);

  /**
   * @brief add the Gyroscope Bias vector baug to the end of the mapping.
   *
   * @param[in] baug: 3x1 gyroscope bias vector baug.
   * @return const Eigen::Vector3d: gyroscope bias vector bg.
   */
  void add_aug_bias(const Eigen::Vector3d& baug);

  /**
   * @brief delete the Gyroscope Bias vector baug according to matrix_idx.
   *
   * @param[in] matrix_idx: the idx in the state matrix where the aug state is
   * located
   */
  void del_aug_bias(int matrix_idx);

  /**
   * @brief Set the Rotation Covariance cov to private member P_.
   *
   * @param[in] cov: covariance matrix for the rotation.
   */
  void set_rotation_covariance(const Eigen::Matrix3d& cov);

  /**
   * @brief Set the Velocity Covariance cov to private member P_.
   *
   * @param[in] cov: covariance matrix for the velocity.
   */
  void set_velocity_covariance(const Eigen::Matrix3d& cov);

  /**
   * @brief Set the Position Covariance cov to private member P_.
   *
   * @param[in] cov: covariance matrix for the position.
   */
  void set_position_covariance(const Eigen::Matrix3d& cov);

  /**
   * @brief Set the Gyroscope Bias Covariance cov to private member P_.
   *
   * @param[in] cov: covariance matrix for the gyroscope bias.
   */
  void set_gyroscope_bias_covariance(const Eigen::Matrix3d& cov);

  /**
   * @brief Set the Accelerometer Bias Covariance cov to private member P_.
   *
   * @param[in] cov: covariance matrix for the accelerometer bias.
   */
  void set_accelerometer_bias_covariance(const Eigen::Matrix3d& cov);

  /**
   * @brief Set the Continuous Noise Covariance cov to private member Qc_.
   *
   * @param[in] cov: covariance matrix for the continuous noise.
   */
  void set_continuous_noise_covariance(const Eigen::MatrixXd& cov);

  /// augment state covariance
  /**
   * @brief add augmented covariance matrix to the end of the mapping.
   *
   * @param[in] cov: augmented covariance matrix.
   */
  void add_aug_cov(const Eigen::Matrix3d& cov);

  /**
   * @brief delete the aug cov matrix according to given matrix_idx
   *
   * @param[in] matrix_idx: the idx in the state matrix where the aug state is
   * located
   */
  void del_aug_cov(int matrix_idx);

  /**
   * @brief set enable bias update boolean variable
   *
   * @param[in] enable_imu_bias_update_: boolean variable to enable/disable bias
   * update
   */
  void set_enable_imu_bias_update(bool enable_imu_bias_update_);

  /**
   * @brief set slip flag to given status
   *
   * @param[in] slip_flag: slip status
   */
  void set_slip_flag(int slip_flag);

  /**
   * @brief Clear the state matrix X_, covariance matrices P_, Qc_ etc.
   */
  void clear();
  /// @} End of Setters

  /// @name Utility functions
  /// @{
  /**
   * @brief copy X_ n times to right-lower side of the larger matrix BigX.
   *
   * @param[in] n: number of copies of X_.
   * @param[in] BigX: larger matrix of states.
   */
  void copy_diag_X(int n, Eigen::MatrixXd& BigX) const;

  /**
   * @brief copy inverse of X_ to right-lower side of the larger matrix BigX.
   *
   * @param[in] n: number of copies of X_.
   * @param[in] BigXinv: larger matrix of inverse of states.
   */
  void copy_diag_Xinv(int n, Eigen::MatrixXd& BigXinv) const;

  /// @} End of Utility functions

  /// @name Overloaded operators
  /// @{
  /**
   * @brief Overloaded operator<< for printing the state.
   *
   * @param[in] os: output stream.
   * @param[in] s: RobotState to be printed.
   * @return std::ostream&: output stream.
   */
  friend std::ostream& operator<<(std::ostream& os, const RobotState& s);
  /// @} End of Overloaded operators

 private:
  // Utility function:
  /**
   * @brief Remove the row and column of the matrix M at the given index.
   *
   * @param[in] M: matrix to be modified.
   * @param[in] index: index of the row and column to be removed.
   * @param[in] move_dim: dimension of the matrix to be decreased.
   */
  void RemoveRowAndColumn(Eigen::MatrixXd& M, int index, int move_dim);

  StateType state_type_ = StateType::WorldCentric;
  Eigen::MatrixXd
      X_;    // Matrix of SE or SEk group represents for robot state.
  Eigen::VectorXd Theta_;        // Matrix of bias respect to X.
  Eigen::MatrixXd P_;            // Matrix of covariance respect to X.
  std::vector<std::shared_ptr<int>>
      column_id_to_corr_map_;    // Mapping from
                                 // column index to
                                 // corresponding
                                 // correction map's value

  double t_;                     // The latest time when the state X_ is updated
  double t_prop_;         // The latest time when the state X_ is propagated
  Eigen::MatrixXd Qc_;    // Continuous noise covariance matrix
  Eigen::Vector3d body_ang_vel_;    // Latest angular velocity
  bool enable_imu_bias_update_ = false;
  int slip_flag_ = 0;               // 0: no slip, 1: slip;
};
}    // namespace state

#endif    // STATE_ROBOT_STATE_H_
