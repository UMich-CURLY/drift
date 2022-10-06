/**
 *  @file   se_3.h
 *  @author Wenzhe Tong
 *  @brief  Header file for various SE(3) functions
 *  @date   September 25, 2018
 **/

#ifndef SE_3_H
#define SE_3_H


#include <cmath>
#include <iostream>

#include <Eigen/Dense>


namespace se_k_3 {

extern const int K;

// Create an abstract class for the SEK3 class
class SEK3 {
 public:
  // Constructor
  SEK3() {}
  SEK3(const Eigen::MatrixXd& X) : X_(X) {}
  SEK3(const Eigen::MatrixXd& R, const Eigen::VectorXd& p) {
    X_.block(0, 0, 3, 3) = R;
    X_.block(0, 3, 3, 1) = p;
    X_.block(3, 0, 1, 3) = Eigen::Vector3d::Zero();
    X_.block(3, 3, 1, 1) = Eigen::Vector3d::Ones();
  }

  // Destructor
  virtual ~SEK3() {}

  // Getters
  virtual Eigen::MatrixXd get_X() const = 0;
  virtual Eigen::MatrixXd get_V() const = 0;
  virtual Eigen::MatrixXd get_Ad() const = 0;
  virtual Eigen::MatrixXd get_Ad_inv() const = 0;
  virtual Eigen::MatrixXd get_Ad_T() const = 0;
  virtual Eigen::MatrixXd get_Ad_inv_T() const = 0;
  virtual Eigen::MatrixXd get_Ad_V() const = 0;
  virtual Eigen::MatrixXd get_Ad_inv_V() const = 0;
  virtual Eigen::MatrixXd get_Ad_T_V() const = 0;
  virtual Eigen::MatrixXd get_Ad_inv_T_V() const = 0;

  // Setters
  virtual void set_X(const Eigen::MatrixXd& X) = 0;
  virtual void set_V(const Eigen::MatrixXd& V) = 0;
  virtual void set_Ad(const Eigen::MatrixXd& Ad) = 0;
  virtual void set_Ad_inv(const Eigen::MatrixXd& Ad_inv) = 0;
  virtual void set_Ad_T(const Eigen::MatrixXd& Ad_T) = 0;
  virtual void set_Ad_inv_T(const Eigen::MatrixXd& Ad_inv_T) = 0;
  virtual void set_Ad_V(const Eigen::MatrixXd& Ad_V) = 0;
  virtual void set_Ad_inv_V(const Eigen::MatrixXd& Ad_inv_V) = 0;
  virtual void set_Ad_T_V(const Eigen::MatrixXd& Ad_T_V) = 0;
  virtual void set_Ad_inv_T_V(const Eigen::MatrixXd& Ad_inv_T_V) = 0;

  // Operators
  virtual SEK3 operator*(const SEK3& X) const = 0;
  virtual SEK3 operator*(const Eigen::MatrixXd& X) const = 0;
  virtual SEK3 operator*(const Eigen::VectorXd& v) const = 0;
  virtual SEK3 operator+(const SEK3& X) const = 0;
  virtual SEK3 operator+(const Eigen::MatrixXd& X) const = 0;
  virtual SEK3 operator+(const Eigen::VectorXd& v) const = 0;
  virtual SEK3 operator-(const SEK3& X) const = 0;

  // Methods
  virtual SEK3 inverse() const = 0;
  virtual SEK3 transpose() const = 0;
  virtual SEK3 log() const = 0;
  virtual SEK3 exp() const = 0;
  virtual SEK3 Adjoint() const = 0;





#endif    // SE_3_H