/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   se_k_3.cpp
 *  @author Wenzhe Tong
 *  @brief  Source file for various SE(3) functions
 *  @date   May 16, 2023
 **/

#include "drift/math/se_k_3.h"

#include <math.h>
#include <vector>


// constructors
namespace math {
SEK3::SEK3() {}

SEK3::SEK3(const Eigen::MatrixXd& X) : X_(X) {}

SEK3::SEK3(const Eigen::MatrixXd& R, const Eigen::VectorXd& p) {
  X_.block(0, 0, 3, 3) = R;
  X_.block(0, 3, 3, 1) = p;
}

SEK3::SEK3(const Eigen::MatrixXd& R, const Eigen::VectorXd& p,
           const Eigen::VectorXd& v) {
  X_.block(0, 0, 3, 3) = R;
  X_.block(0, 3, 3, 1) = p;
  X_.block(0, 4, 3, 1) = v;
}

// getters
const Eigen::MatrixXd SEK3::get_X() const { return X_; }
const Eigen::MatrixXd SEK3::get_R() const { return X_.block(0, 0, 3, 3); }
const Eigen::MatrixXd SEK3::get_p() const { return X_.block(0, 3, 3, 1); }
const Eigen::MatrixXd SEK3::get_v() const { return X_.block(0, 4, 3, 1); }
const int SEK3::get_K() const { return K_; }
const int SEK3::get_dim() const { return X_.rows(); }

// setters
void SEK3::set_X(const Eigen::MatrixXd& X) { X_ = X; }
void SEK3::set_R(const Eigen::MatrixXd& R) { X_.block(0, 0, 3, 3) = R; }
void SEK3::set_p(const Eigen::VectorXd& p) { X_.block(0, 3, 3, 1) = p; }
void SEK3::set_v(const Eigen::MatrixXd& v) { X_.block(0, 4, 3, 1) = v; }
void SEK3::set_K(int K) { K_ = K; }

// operators
SEK3 SEK3::operator*(const SEK3& X) {
  SEK3 Y;
  // TODO: check size of X, if not same, throw error
  Eigen::MatrixXd X1 = X.get_X();
  Y.set_X(this->get_X() * X1);
  Y.set_K(this->get_K());
  return Y;
}

// methods
SEK3 SEK3::inverse() {
  SEK3 Y;
  Y.set_R(this->get_R().transpose());
  Y.set_p(-this->get_R().transpose() * this->get_p());
  Y.set_v(-this->get_R().transpose() * this->get_v());
  Y.set_K(this->get_K());
  return Y;
}
}    // namespace math
