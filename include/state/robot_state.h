/* ----------------------------------------------------------------------------
 * Copyright 2022, Ross Hartley, Wenzhe Tong
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   robot_state.h
 *  @author Ross Hartley, Wenzhe Tong
 *  @brief  Header file for robot state
 *  @date   November 1, 2022
 **/

#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <Eigen/Dense>
#include <iostream>
#include <string>

#include "math/lie_group.h"
#include "state/se_k_3.h"


enum StateType { WorldCentric, BodyCentric };

/**
 * @class RobotState
 *
 * Robot state class containing its pose and augmented states,
 * biases, and covariances.
 */
class RobotState {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// @name Constructors
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

  /**
   * @brief Get state matrix X.
   *
   * @return const Eigen::MatrixXd X: represents for state matrix.
   */
  const Eigen::MatrixXd getX() const;

  /**
   * @brief Get the bias matrix Theta.
   *
   * @return const Eigen::VectorXd Theta: represents for bias matrix.
   */
  const Eigen::VectorXd getTheta() const;

  /**
   * @brief Get the covariance matrix P.
   *
   * @return const Eigen::MatrixXd P: represents for covariance matrix.
   */
  const Eigen::MatrixXd getP() const;

  /**
   * @brief Get the Rotation matrix R.
   *
   * @return const Eigen::MatrixXd R: represents for rotation matrix.
   */
  const Eigen::Matrix3d getRotation() const;

  /**
   * @brief Get the Velocity matrix V.
   *
   * @return const Eigen::Vector3d V: represents for velocity matrix.
   */
  const Eigen::Vector3d getVelocity() const;

  /**
   * @brief Get the Position matrix p.
   *
   * @return const Eigen::Vector3d p: represents for position matrix.
   */
  const Eigen::Vector3d getPosition() const;

  /**
   * @brief Get the gyroscope bias matrix b.
   *
   * @return const Eigen::Vector3d b: represents for gyroscope bias matrix.
   */
  const Eigen::Vector3d getGyroscopeBias() const;

  /**
   * @brief Get the a 3 by 1 vector from the state
   *
   * @return const Eigen::Vector3d
   */
  const Eigen::Vector3d getVector(int index) const;

  /**
   * @brief Get the accelerometer bias matrix b.
   *
   * @return const Eigen::Vector3d b: represents for accelerometer bias
   * matrix.
   */
  const Eigen::Vector3d getAccelerometerBias() const;

  /**
   * @brief Get the rotation covariance matrix P.
   *
   * @return const Eigen::Matrix3d P: represents for rotation covariance matrix.
   */
  const Eigen::Matrix3d getRotationCovariance() const;

  /**
   * @brief Get the velocity covariance matrix P.
   *
   * @return const Eigen::Matrix3d P: represents for velocity covariance matrix.
   */
  const Eigen::Matrix3d getVelocityCovariance() const;

  /**
   * @brief Get the position covariance matrix P.
   *
   * @return const Eigen::Matrix3d P: represents for position covariance matrix.
   */
  const Eigen::Matrix3d getPositionCovariance() const;

  /**
   * @brief Get the gyroscope bias covariance matrix P.
   *
   * @return const Eigen::Matrix3d P: represents for gyroscope bias covariance
   * matrix.
   */
  const Eigen::Matrix3d getGyroscopeBiasCovariance() const;

  /**
   * @brief Get the accelerometer bias covariance matrix P.
   *
   * @return const Eigen::Matrix3d P: represents for accelerometer bias
   * covariance matrix.
   */
  const Eigen::Matrix3d getAccelerometerBiasCovariance() const;

  /**
   * @brief Get the vector of augmented states mapping. Each mapping means
   * certain types of augmented states(contact, landmark, additional IMU
   * measurement etc.) restores the mapping from augmented states index(e.g.
   * contact 1, 2, 3, 4) to the state matrix index(e.g. 6th, 7th, 8th, 9th
   * columns in state matrix X).
   *
   * @return std::vector<std::map<int, int>>: vector of different types of index
   * mapping from augmented states to state matrix index.
   */
  std::vector<std::map<int, int>> get_augmented_maps();

  /**
   * @brief Add an empty mapping in the mapping vector.
   *
   * @return int: the index of the mapping in the mapping vector idx_maps_.
   */
  int add_augmented_map();

  /**
   * @brief Get the augmented map at certain index.
   *
   * @param[in] idx_map: the index of the mapping in the mapping vector
   * idx_maps_.
   * @return std::map<int, int>: represents for the mapping from augmented
   * states to the state matrix index.
   */
  std::map<int, int> get_augmented_map(int idx_map);

  /**
   * @brief Add the augmented state to the last of the certain(idx_map-th)
   * mapping in the idx_maps_ vector.
   *
   * @param[in] idx_map: the index of the mapping in the mapping vector
   * idx_maps_.
   * @param[in] aug: Augmented state to be added to the index mapping.
   */
  int add_aug_state(int idx_map, const Eigen::Vector3d& aug);

  /**
   * @brief Set the augmented to the certain position(idx_state-th) of the
   * certain(idx_map-th) mapping in the idx_maps_ vector.
   *
   * @param[in] idx_map: index of the mapping in the mapping vector idx_maps_.
   * @param[in] idx_state: index of the augmented state in the mapping.
   * @param[in] aug: augmented state to be set.
   */
  void set_aug_state(int idx_map, int idx_state, const Eigen::Vector3d& aug);

  /**
   * @brief Delete certain augmented state(idx_state-th) from the
   * certain(idx_map-th) mapping
   *
   * @param[in] idx_map: index of the mapping in the mapping vector idx_maps_.
   * @param[in] idx_state: index of the augmented state in the mapping.
   */
  void del_aug_state(int idx_map, int idx_state);

  /**
   * @brief Get the augmented state vector giving idx_state and idx_map.
   *
   * @param[in] idx_map: index of the mapping in the mapping vector idx_maps_.
   * @param[in] idx_state: index of the augmented state in the mapping.
   * @return const Eigen::Vector3d: augmented state vector.
   */
  const Eigen::Vector3d get_aug_state(int idx_map, int idx_state);

  /**
   * @brief get the dimension of the state matrix X.
   *
   * @return const int: dimension of the state matrix X.
   */
  const int dimX() const;

  /**
   * @brief get the dimension of the augmented state matrix X.
   *
   * @return const int: dimension of the augmented state matrix X.
   */
  const int dimTheta() const;

  /**
   * @brief get the dimension of the covariance matrix P.
   *
   * @return const int: dimension of the augmented covariance matrix P.
   */
  const int dimP() const;

  /**
   * @brief get the the state type from: {WorldCentric, BodyCentric}
   *
   * @return const StateType: dimension of the augmented covariance matrix P.
   */
  const StateType getStateType() const;

  /**
   * @brief Get the WorldCentric state matrix X.
   *
   * @return const Eigen::MatrixXd: state matrix X in world frame.
   */
  const Eigen::MatrixXd getWorldX() const;

  /**
   * @brief Get the WorldCentric Rotation matrix R.
   *
   * @return const Eigen::Matrix3d: state matrix R in the world frame.
   */
  const Eigen::Matrix3d getWorldRotation() const;

  /**
   * @brief Get the WorldCentric Velocity vector v.
   *
   * @return const Eigen::Vector3d: velocity vector v in the world frame.
   */
  const Eigen::Vector3d getWorldVelocity() const;

  /**
   * @brief Get the WorldCentric Position vector p.
   *
   * @return const Eigen::Vector3d: position vector p in the world frame.
   */
  const Eigen::Vector3d getWorldPosition() const;

  /**
   * @brief Get the BodyCentric state matrix X.
   *
   * @return const Eigen::MatrixXd: state matrix X in body frame.
   */
  const Eigen::MatrixXd getBodyX() const;

  /**
   * @brief Get the BodyCentric Rotation matrix R.
   *
   * @return const Eigen::Matrix3d: state matrix R in the body frame.
   */
  const Eigen::Matrix3d getBodyRotation() const;

  /**
   * @brief Get the BodyCentric Velocity vector v.
   *
   * @return const Eigen::Vector3d: velocity vector v in the body frame.
   */
  const Eigen::Vector3d getBodyVelocity() const;

  /**
   * @brief Get the BodyCentric Position vector p.
   *
   * @return const Eigen::Vector3d: position vector p in the body frame.
   */
  const Eigen::Vector3d getBodyPosition() const;

  /**
   * @brief Set the RobotState whole matrix.
   *
   * @param[in]: X: state matrix X.
   */
  void setX(const Eigen::MatrixXd& X);

  /**
   * @brief Set the RobotState Bias matrix P.
   *
   * @param[in] P: bias matrix P.
   */
  void setP(const Eigen::MatrixXd& P);

  /**
   * @brief Set the RobotState covariance matrix Theta.
   *
   * @param[in] Theta: covariance matrix Theta.
   */
  void setTheta(const Eigen::VectorXd& Theta);

  /**
   * @brief Set the Rotation matrix R in RobotState.
   *
   * @param[in] R: 3x3 rotation matrix R.
   */
  void setRotation(const Eigen::Matrix3d& R);

  /**
   * @brief Set the Velocity vector v in RoboState.
   *
   * @param[in] v: 3x1 velocity vector v.
   */
  void setVelocity(const Eigen::Vector3d& v);

  /**
   * @brief Set the Position vector p in RoboState.
   *
   * @param[in] p: 3x1 position vector p.
   */
  void setPosition(const Eigen::Vector3d& p);

  /**
   * @brief Set the Gyroscope Bias to bias matrix P.
   *
   * @param[in] bg: 3x1 gyroscope bias vector bg.
   */
  void setGyroscopeBias(const Eigen::Vector3d& bg);

  /**
   * @brief Set the Accelerometer Bias to bias matrix P.
   *
   * @param[in] ba: 3x1 accelerometer bias vector ba.
   */
  void setAccelerometerBias(const Eigen::Vector3d& ba);

  /**
   * @brief add the Gyroscope Bias vector baug to the end of the mapping.
   *
   * @param[in] idx_map: index of the mapping in the mapping vector idx_maps_.
   * @return const Eigen::Vector3d: gyroscope bias vector bg.
   */
  int add_aug_bias(int idx_map, const Eigen::Vector3d& baug);

  /**
   * @brief set the Gyroscope Bias vector baug to the certain
   * position(idx_state-th) of the certain(idx_map-th) mapping in the idx_maps_
   * vector.
   *
   * @param[in] idx_map: index of the mapping in the mapping vector idx_maps_.
   * @param[in] idx_state: index of the augmented state in the mapping.
   * @param[in] baug: gyroscope bias vector baug.
   */
  void set_aug_bias(int idx_map, int idx_state, const Eigen::Vector3d& baug);

  /**
   * @brief delete the Gyroscope Bias vector baug from the certain(idx_map-th)
   * mapping in the idx_maps_ vector.
   *
   * @param[in] idx_map: index of the mapping in the mapping vector idx_maps_.
   * @param[in] idx_state: index of the augmented state in the mapping.
   */
  void del_aug_bias(int idx_map, int idx_state);

  /**
   * @brief Get the Gyroscope Bias vector baug from the certain(idx_map-th)
   * mapping in the idx_maps_ vector.
   *
   * @param[in] idx_map: index of the mapping in the mapping vector idx_maps_.
   * @param[in] idx_state: index of the augmented state in the mapping.
   * @return const Eigen::Vector3d: gyroscope bias vector baug.
   */
  const Eigen::Vector3d get_aug_bias(int idx_map, int idx_state);

  /**
   * @brief Set the Rotation Covariance cov to private member P_.
   *
   * @param[in] cov: covariance matrix for the rotation.
   */
  void setRotationCovariance(const Eigen::Matrix3d& cov);

  /**
   * @brief Set the Velocity Covariance cov to private member P_.
   *
   * @param[in] cov: covariance matrix for the velocity.
   */
  void setVelocityCovariance(const Eigen::Matrix3d& cov);

  /**
   * @brief Set the Position Covariance cov to private member P_.
   *
   * @param[in] cov: covariance matrix for the position.
   */
  void setPositionCovariance(const Eigen::Matrix3d& cov);

  /**
   * @brief Set the Gyroscope Bias Covariance cov to private member P_.
   *
   * @param[in] cov: covariance matrix for the gyroscope bias.
   */
  void setGyroscopeBiasCovariance(const Eigen::Matrix3d& cov);

  /**
   * @brief Set the Accelerometer Bias Covariance cov to private member P_.
   *
   * @param[in] cov: covariance matrix for the accelerometer bias.
   */
  void setAccelerometerBiasCovariance(const Eigen::Matrix3d& cov);


  /// augment state covariance
  /**
   * @brief add augmented covariance matrix to the end of the mapping.
   *
   * @param[in] idx_map: index of the mapping in the mapping vector idx_maps_.
   * @param[in] cov: augmented covariance matrix.
   * @return int: index of the augmented covariance matrix in the mapping.
   */
  int add_aug_cov(int idx_map, const Eigen::Matrix3d& cov);

  /**
   * @brief Set the aug cov matrix to the certain position(idx_state-th) of the
   * certain(idx_map-th) mapping in the idx_maps_ vector.
   *
   * @param[in] idx_map: index of the mapping in the mapping vector idx_maps_.
   * @param[in] idx_state: index of the augmented state in the mapping.
   * @param[in] cov: augmented covariance matrix.
   */
  void set_aug_cov(int idx_map, int idx_state, const Eigen::Matrix3d& cov);

  /**
   * @brief delete the aug cov matrix from the certain(idx_map-th) mapping in
   * the idx_maps_ vector.
   *
   * @param[in] idx_map: index of the mapping in the mapping vector idx_maps_.
   * @param[in] idx_state: index of the augmented state in the mapping.
   */
  void del_aug_cov(int idx_map, int idx_state);

  /**
   * @brief Get the aug cov matrix from the certain(idx_map-th) mapping in the
   * idx_maps_ vector.
   *
   * @param[in] idx_map: index of the mapping in the mapping vector idx_maps_.
   * @param[in] idx_state: index of the augmented state in the mapping.
   * @return const Eigen::Matrix3d: augmented covariance matrix.
   */
  const Eigen::Matrix3d get_aug_cov(int idx_map, int idx_state);

  /**
   * @brief copy X_ n times to right-lower side of the larger matrix BigX.
   *
   * @param[in] n: number of copies of X_.
   * @param[in] BigX: larger matrix of states.
   */
  void copyDiagX(int n, Eigen::MatrixXd& BigX) const;

  /**
   * @brief copy inverse of X_ to right-lower side of the larger matrix BigX.
   *
   * @param[in] n: number of copies of X_.
   * @param[in] BigXinv: larger matrix of inverse of states.
   */
  void copyDiagXinv(int n, Eigen::MatrixXd& BigXinv) const;

  /**
   * @brief compute the inverse of the state matrix X_.
   *
   * @return const Eigen::MatrixXd: inverse of the state matrix X_.
   */
  const Eigen::MatrixXd Xinv() const;

  friend std::ostream& operator<<(std::ostream& os, const RobotState& s);

 private:
  StateType state_type_ = StateType::WorldCentric;
  Eigen::MatrixXd X_;
  Eigen::VectorXd Theta_;
  Eigen::MatrixXd P_;
  std::vector<std::map<int, int>> idx_maps_;
};

#endif
