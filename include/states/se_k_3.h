/**
 *  @file   se_k_3.h
 *  @author Wenzhe Tong
 *  @brief  Header file for various SE(3) functions
 *  @date   October 5th, 2022
 **/

#ifndef SE_K_3_H
#define SE_K_3_H


#include <cmath>
#include <iostream>

#include <Eigen/Dense>


namespace se_k_3 {

class SEK3 {
 public:
  // Member variables
  int K_ = 4;
  Eigen::MatrixXd X_ = Eigen::MatrixXd::Identity(K_, K_);

  // Constructor
  SEK3() {};
  SEK3(const Eigen::MatrixXd& X) : X_(X) {};
  SEK3(const Eigen::MatrixXd& R, const Eigen::VectorXd& p) {};
  SEK3(const Eigen::MatrixXd& R, const Eigen::VectorXd& p, const Eigen::VectorXd& v) {};

  // Destructor
  ~SEK3() {}

  // Getters
  Eigen::MatrixXd get_X();
  Eigen::MatrixXd get_V();
  // virtual Eigen::MatrixXd get_Ad() const = 0;
  // virtual Eigen::MatrixXd get_Ad_inv() const = 0;
  // virtual Eigen::MatrixXd get_Ad_T() const = 0;
  // virtual Eigen::MatrixXd get_Ad_inv_T() const = 0;
  // virtual Eigen::MatrixXd get_Ad_V() const = 0;
  // virtual Eigen::MatrixXd get_Ad_inv_V() const = 0;
  // virtual Eigen::MatrixXd get_Ad_T_V() const = 0;
  // virtual Eigen::MatrixXd get_Ad_inv_T_V() const = 0;

  // Setters
  void set_R(const Eigen::MatrixXd& R);
  void set_X(const Eigen::MatrixXd& X);
  void set_V(const Eigen::MatrixXd& V);
  // virtual void set_Ad(const Eigen::MatrixXd& Ad) = 0;
  // virtual void set_Ad_inv(const Eigen::MatrixXd& Ad_inv) = 0;
  // virtual void set_Ad_T(const Eigen::MatrixXd& Ad_T) = 0;
  // virtual void set_Ad_inv_T(const Eigen::MatrixXd& Ad_inv_T) = 0;
  // virtual void set_Ad_V(const Eigen::MatrixXd& Ad_V) = 0;
  // virtual void set_Ad_inv_V(const Eigen::MatrixXd& Ad_inv_V) = 0;
  // virtual void set_Ad_T_V(const Eigen::MatrixXd& Ad_T_V) = 0;
  // virtual void set_Ad_inv_T_V(const Eigen::MatrixXd& Ad_inv_T_V) = 0;

  // Operators
  virtual SEK3 operator*(const SEK3& X) const = 0;
  virtual SEK3 operator*(const Eigen::MatrixXd& X) const = 0;
  virtual SEK3 operator*(const Eigen::VectorXd& v) const = 0;
  virtual SEK3 operator*(const Eigen::MatrixXd& T) const = 0;
  // virtual SEK3 operator/(const SEK3& X) const = 0;
  // virtual SEK3 operator/(const Eigen::MatrixXd& X) const = 0;
  // virtual SEK3 operator/(const Eigen::VectorXd& v) const = 0;
  // virtual SEK3 operator/(const Eigen::MatrixXd& T) const = 0;
  // virtual SEK3 operator
  virtual SEK3 operator<<(const SEK3& X) const = 0;
  virtual SEK3 operator<<(const Eigen::MatrixXd& X) const = 0;
  virtual SEK3 operator<<(const Eigen::VectorXd& v) const = 0;
  virtual SEK3 operator<<(const Eigen::MatrixXd& T) const = 0;

  // Methods
  virtual SEK3 inverse() const = 0;
  virtual SEK3 transpose() const = 0;
  virtual SEK3 log() const = 0;
  virtual SEK3 exp() const = 0;
  virtual SEK3 Adjoint() const = 0;

}    // class SEK3

}    // namespace se_k_3


#endif    // SE_3_H