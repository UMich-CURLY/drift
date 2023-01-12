/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   robot_state.cpp
 *  @author Wenzhe Tong, Tingjun Li
 *  @brief  Source file for RobotState
 *  @date   Nov 1st, 2022
 **/

#include "state/robot_state.h"

using namespace std;

// Default constructor
RobotState::RobotState()
    : X_(Eigen::MatrixXd::Identity(5, 5)),
      Theta_(Eigen::MatrixXd::Zero(6, 1)),
      P_(Eigen::MatrixXd::Identity(15, 15)),
      Qc_(Eigen::MatrixXd::Identity(15, 15)) {}

// Initialize with X
RobotState::RobotState(const Eigen::MatrixXd& X)
    : X_(X), Theta_(Eigen::MatrixXd::Zero(6, 1)) {
  P_ = Eigen::MatrixXd::Identity(3 * this->dimX() + this->dimTheta() - 6,
                                 3 * this->dimX() + this->dimTheta() - 6);
  Qc_ = Eigen::MatrixXd::Identity(this->dimP(), this->dimP());
}

// Initialize with X and Theta
RobotState::RobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta)
    : X_(X), Theta_(Theta) {
  P_ = Eigen::MatrixXd::Identity(3 * this->dimX() + this->dimTheta() - 6,
                                 3 * this->dimX() + this->dimTheta() - 6);
  Qc_ = Eigen::MatrixXd::Identity(this->dimP(), this->dimP());
}

// Initialize with X, Theta and P
RobotState::RobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta,
                       const Eigen::MatrixXd& P)
    : X_(X), Theta_(Theta), P_(P) {
  Qc_ = Eigen::MatrixXd::Identity(this->dimP(), this->dimP());
}

// Initialize with SEK3
RobotState::RobotState(SEK3& X)
    : X_(X.get_X()), Theta_(Eigen::MatrixXd::Zero(3 * (this->dimX() - 3), 1)) {
  P_ = Eigen::MatrixXd::Identity(3 * this->dimX() + this->dimTheta() - 6,
                                 3 * this->dimX() + this->dimTheta() - 6);
  Qc_ = Eigen::MatrixXd::Identity(this->dimP(), this->dimP());
}

// Initialize with SEK3 and Theta
RobotState::RobotState(SEK3& X, const Eigen::VectorXd& Theta)
    : X_(X.get_X()), Theta_(Theta) {
  P_ = Eigen::MatrixXd::Identity(3 * this->dimX() + this->dimTheta() - 6,
                                 3 * this->dimX() + this->dimTheta() - 6);
  Qc_ = Eigen::MatrixXd::Identity(this->dimP(), this->dimP());
}

// Initialize with SEK3, Theta and P
RobotState::RobotState(SEK3& X, const Eigen::VectorXd& Theta,
                       const Eigen::MatrixXd& P)
    : X_(X.get_X()), Theta_(Theta), P_(P) {
  Qc_ = Eigen::MatrixXd::Identity(this->dimP(), this->dimP());
}

// getters
const Eigen::MatrixXd RobotState::get_X() const { return X_; }
const Eigen::VectorXd RobotState::get_theta() const { return Theta_; }
const Eigen::MatrixXd RobotState::get_P() const { return P_; }

Eigen::MatrixXd RobotState::get_continuous_noise_covariance() const {
  return Qc_;
}
const Eigen::Matrix3d RobotState::get_rotation() const {
  return X_.block<3, 3>(0, 0);
}
const Eigen::Vector3d RobotState::get_velocity() const {
  return X_.block<3, 1>(0, 3);
}
const Eigen::Vector3d RobotState::get_position() const {
  return X_.block<3, 1>(0, 4);
}
const Eigen::Vector3d RobotState::get_vector(int index) const {
  return X_.block<3, 1>(0, index);
}
// const Eigen::Vector3d RobotState::getAugState(std::string key) {
//     int idx = SEK3::get_aug_index(key) - 1;
//     return X_.block<3,1>(0,idx);
// }

const Eigen::Vector3d RobotState::get_gyroscope_bias() const {
  return Theta_.block<3, 1>(0, 0);
}
const Eigen::Vector3d RobotState::get_accelerometer_bias() const {
  return Theta_.block<3, 1>(3, 0);
}
// const Eigen::Vector3d RobotState::getAugStateBias(std::string key) {
//     int idx = SEK3::get_aug_index(key) - 1;
//     return Theta_.block<3,1>(idx,0);
// }

const Eigen::Matrix3d RobotState::get_rotation_covariance() const {
  return P_.block<3, 3>(0, 0);
}
const Eigen::Matrix3d RobotState::get_velocity_covariance() const {
  return P_.block<3, 3>(3, 3);
}
const Eigen::Matrix3d RobotState::get_position_covariance() const {
  return P_.block<3, 3>(6, 6);
}
const Eigen::Matrix3d RobotState::get_gyroscope_bias_covariance() const {
  return P_.block<3, 3>(9, 9);
}
const Eigen::Matrix3d RobotState::get_accelerometer_bias_covariance() const {
  return P_.block<3, 3>(12, 12);
}

const double RobotState::get_time() const { return t_; }

const double RobotState::get_propagate_time() const { return t_prop_; }

int RobotState::add_aug_state(const Eigen::Vector3d& aug,
                              const Eigen::Matrix3d& cov,
                              const Eigen::Matrix3d& noise_cov) {
  Eigen::MatrixXd X_aug
      = Eigen::MatrixXd::Identity(this->dimX() + 1, this->dimX() + 1);
  X_aug.block(0, 0, this->dimX(), this->dimX()) = X_;
  X_aug.block(0, this->dimX(), 3, 1) = aug;
  X_ = X_aug;

  Eigen::MatrixXd P_aug
      = Eigen::MatrixXd::Zero(this->dimP() + 3, this->dimP() + 3);
  P_aug.block(0, 0, this->dimP(), this->dimP()) = P_;
  P_aug.block<3, 3>(this->dimP(), this->dimP()) = cov;
  P_ = P_aug;

  // Augment Qc
  Eigen::MatrixXd Qc_aug
      = Eigen::MatrixXd::Zero(this->dimQc() + 3, this->dimQc() + 3);
  Qc_aug.block(0, 0, this->dimQc(), this->dimQc()) = Qc_;
  Qc_aug.block<3, 3>(this->dimQc(), this->dimQc()) = noise_cov;
  Qc_ = Qc_aug;

  return this->dimX() - 1;
}

void RobotState::set_aug_state(int matrix_idx, const Eigen::Vector3d& aug) {
  X_.block<3, 1>(0, matrix_idx) = aug;
}

void RobotState::del_aug_state(int matrix_idx) {
  Eigen::MatrixXd X_aug
      = Eigen::MatrixXd::Identity(this->dimX() - 1, this->dimX() - 1);
  X_aug.block(0, 0, matrix_idx, matrix_idx)
      = X_.block(0, 0, matrix_idx, matrix_idx);
  X_aug.block(matrix_idx, matrix_idx, this->dimX() - matrix_idx - 1,
              this->dimX() - matrix_idx - 1)
      = X_.block(matrix_idx + 1, matrix_idx + 1, this->dimX() - matrix_idx - 1,
                 this->dimX() - matrix_idx - 1);
  X_ = X_aug;

  this->del_aug_cov(matrix_idx);
  this->del_aug_bias(matrix_idx);
  this->del_aug_noise_cov(matrix_idx);
}

const Eigen::Vector3d RobotState::get_aug_state(int matrix_idx) {
  return X_.block<3, 1>(0, matrix_idx);
}

const int RobotState::dimX() const { return X_.cols(); }
const int RobotState::dimTheta() const { return Theta_.rows(); }
const int RobotState::dimP() const { return P_.cols(); }
const int RobotState::dimQc() const { return Qc_.cols(); }

const StateType RobotState::get_state_type() const { return state_type_; }

const Eigen::MatrixXd RobotState::get_world_X() const {
  if (state_type_ == StateType::WorldCentric) {
    return this->get_X();
  } else {
    return this->Xinv();
  }
}

const Eigen::Matrix3d RobotState::get_world_rotation() const {
  if (state_type_ == StateType::WorldCentric) {
    return this->get_rotation();
  } else {
    return this->get_rotation().transpose();
  }
}

const Eigen::Vector3d RobotState::get_world_velocity() const {
  if (state_type_ == StateType::WorldCentric) {
    return this->get_velocity();
  } else {
    return -this->get_rotation().transpose() * this->get_velocity();
  }
}

const Eigen::Vector3d RobotState::get_world_position() const {
  if (state_type_ == StateType::WorldCentric) {
    return this->get_position();
  } else {
    return -this->get_rotation().transpose() * this->get_position();
  }
}

const Eigen::MatrixXd RobotState::get_body_X() const {
  if (state_type_ == StateType::BodyCentric) {
    return this->get_X();
  } else {
    return this->Xinv();
  }
}

const Eigen::Matrix3d RobotState::get_body_rotation() const {
  if (state_type_ == StateType::BodyCentric) {
    return this->get_rotation();
  } else {
    return this->get_rotation().transpose();
  }
}

const Eigen::Vector3d RobotState::get_body_velocity() const {
  if (state_type_ == StateType::BodyCentric) {
    return this->get_velocity();
  } else {
    return -this->get_rotation().transpose() * this->get_velocity();
  }
}

const Eigen::Vector3d RobotState::get_body_position() const {
  if (state_type_ == StateType::BodyCentric) {
    return this->get_position();
  } else {
    return -this->get_rotation().transpose() * this->get_position();
  }
}

// setters
void RobotState::set_X(const Eigen::MatrixXd& X) { X_ = X; }
void RobotState::set_theta(const Eigen::VectorXd& Theta) { Theta_ = Theta; }
void RobotState::set_P(const Eigen::MatrixXd& P) { P_ = P; }
void RobotState::set_continuous_noise_covariance(const Eigen::MatrixXd& cov) {
  Qc_ = cov;
}

void RobotState::set_rotation(const Eigen::Matrix3d& R) {
  X_.block<3, 3>(0, 0) = R;
}
void RobotState::set_velocity(const Eigen::Vector3d& v) {
  X_.block<3, 1>(0, 3) = v;
}
void RobotState::set_position(const Eigen::Vector3d& p) {
  X_.block<3, 1>(0, 4) = p;
}

void RobotState::set_gyroscope_bias(const Eigen::Vector3d& bg) {
  Theta_.block<3, 1>(0, 0) = bg;
}
void RobotState::set_accelerometer_bias(const Eigen::Vector3d& ba) {
  Theta_.block<3, 1>(3, 0) = ba;
}


void RobotState::add_aug_bias(int matrix_idx, const Eigen::Vector3d& baug) {
  Eigen::VectorXd Theta_aug = Eigen::VectorXd::Zero(Theta_.rows() + 3);
  Theta_aug.block(0, 0, 1, Theta_.rows()) = Theta_;
  Theta_aug.block<3, 1>(0, Theta_.rows()) = baug;
  Theta_ = Theta_aug;
}


void RobotState::del_aug_bias(int matrix_idx) {
  Eigen::VectorXd Theta_aug = Eigen::VectorXd::Zero(Theta_.rows() - 3);
  Theta_aug.block(0, 0, 1, Theta_.rows() - 3)
      = Theta_.block(0, 0, 1, Theta_.rows() - 3);
  Theta_ = Theta_aug;
}


const Eigen::Vector3d RobotState::get_aug_bias(int matrix_idx) {
  return Theta_.block<3, 1>(0, Theta_.rows() - 3);
}

void RobotState::set_rotation_covariance(const Eigen::Matrix3d& cov) {
  P_.block<3, 3>(0, 0) = cov;
}
void RobotState::set_velocity_covariance(const Eigen::Matrix3d& cov) {
  P_.block<3, 3>(3, 3) = cov;
}
void RobotState::set_position_covariance(const Eigen::Matrix3d& cov) {
  P_.block<3, 3>(6, 6) = cov;
}
void RobotState::set_gyroscope_bias_covariance(const Eigen::Matrix3d& cov) {
  P_.block<3, 3>(9, 9) = cov;
}
void RobotState::set_accelerometer_bias_covariance(const Eigen::Matrix3d& cov) {
  P_.block<3, 3>(12, 12) = cov;
}
void RobotState::set_time(const double t) { t_ = t; }

void RobotState::set_propagate_time(const double t) { t_prop_ = t; }

void RobotState::add_aug_cov(const Eigen::Matrix3d& cov) {
  int dim = this->dimP();
  /// TODO: Add conservative resize to avoid construction of new matrix
  Eigen::MatrixXd P_aug = Eigen::MatrixXd::Zero(dim + 3, dim + 3);
  P_aug.block(0, 0, dim, dim) = P_;
  P_aug.block<3, 3>(dim, dim) = cov;
  P_ = P_aug;
}

void RobotState::del_aug_cov(int matrix_idx) {
  int dim = this->dimP();
  /// TODO: Add conservative resize to avoid construction of new matrix
  Eigen::MatrixXd P_aug = Eigen::MatrixXd::Zero(dim - 3, dim - 3);
  int idx_cov = matrix_idx * 3 - 12;
  P_aug.block(0, 0, idx_cov, idx_cov) = P_.block(0, 0, idx_cov, idx_cov);
  P_aug.block(idx_cov, idx_cov, dim - idx_cov - 3, dim - idx_cov - 3)
      = P_.block(idx_cov + 3, idx_cov + 3, dim - idx_cov - 3,
                 dim - idx_cov - 3);
  P_ = P_aug;
}

void RobotState::del_aug_noise_cov(int matrix_idx) {
  int dim = this->dimQc();
  /// TODO: Add conservative resize to avoid construction of new matrix
  Eigen::MatrixXd Qc_aug = Eigen::MatrixXd::Zero(dim - 3, dim - 3);
  int idx_cov = matrix_idx * 3 - 12;
  Qc_aug.block(0, 0, idx_cov, idx_cov) = Qc_.block(0, 0, idx_cov, idx_cov);
  Qc_aug.block(idx_cov, idx_cov, dim - idx_cov - 3, dim - idx_cov - 3)
      = Qc_.block(idx_cov + 3, idx_cov + 3, dim - idx_cov - 3,
                  dim - idx_cov - 3);
  Qc_ = Qc_aug;
}

const Eigen::Matrix3d RobotState::get_aug_cov(int matrix_idx) {
  int idx_cov = matrix_idx * 3 - 12;
  return P_.block<3, 3>(idx_cov, idx_cov);
}

void RobotState::copy_diag_X(int n, Eigen::MatrixXd& BigX) const {
  int dimX = this->dimX();
  for (int i = 0; i < n; ++i) {
    int startIndex = BigX.rows();
    BigX.conservativeResize(startIndex + dimX, startIndex + dimX);
    BigX.block(startIndex, 0, dimX, startIndex)
        = Eigen::MatrixXd::Zero(dimX, startIndex);
    BigX.block(0, startIndex, startIndex, dimX)
        = Eigen::MatrixXd::Zero(startIndex, dimX);
    BigX.block(startIndex, startIndex, dimX, dimX) = X_;
  }
  return;
}

void RobotState::copy_diag_Xinv(int n, Eigen::MatrixXd& BigXinv) const {
  int dimX = this->dimX();
  Eigen::MatrixXd Xinv = this->Xinv();
  for (int i = 0; i < n; ++i) {
    int startIndex = BigXinv.rows();
    BigXinv.conservativeResize(startIndex + dimX, startIndex + dimX);
    BigXinv.block(startIndex, 0, dimX, startIndex)
        = Eigen::MatrixXd::Zero(dimX, startIndex);
    BigXinv.block(0, startIndex, startIndex, dimX)
        = Eigen::MatrixXd::Zero(startIndex, dimX);
    BigXinv.block(startIndex, startIndex, dimX, dimX) = Xinv;
  }
  return;
}

const Eigen::MatrixXd RobotState::Xinv() const {
  int dimX = this->dimX();
  Eigen::MatrixXd Xinv = Eigen::MatrixXd::Identity(dimX, dimX);
  Eigen::Matrix3d RT = X_.block<3, 3>(0, 0).transpose();
  Xinv.block<3, 3>(0, 0) = RT;
  for (int i = 3; i < dimX; ++i) {
    Xinv.block<3, 1>(0, i) = -RT * X_.block<3, 1>(0, i);
  }
  return Xinv;
}


ostream& operator<<(ostream& os, const RobotState& s) {
  os << "--------- Robot State -------------" << endl;
  os << "X:\n" << s.X_ << endl << endl;
  os << "Theta:\n" << s.Theta_ << endl << endl;
  // os << "P:\n" << s.P_ << endl;
  os << "-----------------------------------";
  return os;
}
