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
#include <map>
#include <string>
#include <vector>

#include <Eigen/Dense>


namespace se_k_3 {

class SEK3 {
 public:
  // Constructor
  SEK3();
  SEK3(const Eigen::MatrixXd& X);
  SEK3(const Eigen::MatrixXd& R, const Eigen::VectorXd& p);
  SEK3(const Eigen::MatrixXd& R, const Eigen::VectorXd& p,
       const Eigen::VectorXd& v);

  // Destructor
  ~SEK3() {}

  // Getters
  Eigen::MatrixXd get_X();
  Eigen::MatrixXd get_R();
  Eigen::MatrixXd get_p();
  Eigen::MatrixXd get_v();

  Eigen::MatrixXd get_aug(std::string key);
  std::vector<std::string> get_aug_keys();
  int get_aug_index(std::string key);
  int get_dim();

  // Setters
  void set_K(int K);
  void set_X(const Eigen::MatrixXd& X);
  void set_R(const Eigen::MatrixXd& R);
  void set_p(const Eigen::VectorXd& p);
  void set_v(const Eigen::MatrixXd& v);

  // Setters - aug state
  void set_aug(std::string key, const Eigen::VectorXd& aug);

  // detele aug state
  void del_aug(std::string key);

  // Operators
  SEK3 operator*(const SEK3& X);

  // Methods
  SEK3 inverse();
  // SEK3 log();
  // SEK3 exp();
  // SEK3 Adjoint();

 private:
  // Member variables
  int K_ = 5;
  Eigen::MatrixXd X_ = Eigen::MatrixXd::Identity(K_, K_);
  std::map<std::string, int> map_aug_;

};    // class SEK3
ß
}    // namespace se_k_3


#endif    // SE_3_H