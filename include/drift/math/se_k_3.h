/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   se_k_3.h
 *  @author Wenzhe Tong
 *  @brief  Header file for various SE(3) functions
 *  @date   October 16, 2023
 **/

#ifndef MATH_SE_K_3_H
#define MATH_SE_K_3_H


#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Dense>


namespace math {
/**
 * @class SEK3
 * @brief SEk(3) group class
 *
 * SEk(3) is a group of SE(3) with k additional states
 */
class SEK3 {
 public:
  /// @name Constructors
  /// @{
  /**
   * @brief Default constructor.
   */
  SEK3();

  /**
   * @brief Construct a new SEK3 object from a matrix.
   *
   * @param[in] X:
   */
  SEK3(const Eigen::MatrixXd& X);

  /**
   * @brief Construct a new SEK3 object from a rotation matrix and a position
   * vector.
   *
   * @param[in] R: Rotation matrix
   * @param[in] p: Position vector
   */
  SEK3(const Eigen::MatrixXd& R, const Eigen::VectorXd& p);

  /**
   * @brief Construct a new SEK3 object from a rotation matrix, a position
   * vector, and a velocity vector.
   *
   * @param[in] R: Rotation matrix
   * @param[in] p: Position vector
   * @param[in] v: Velocity vector
   */
  SEK3(const Eigen::MatrixXd& R, const Eigen::VectorXd& p,
       const Eigen::VectorXd& v);
  /// @}

  ///@name Destructor
  /// @{
  /**
   * @brief Destroy the SEK3 object
   */
  ~SEK3() {}
  /// @}

  ///@name Getters
  /// @{
  /**
   * @brief Get the states matrix X.
   *
   * @return Eigen::MatrixXd: States matrix
   */
  const Eigen::MatrixXd get_X() const;

  /**
   * @brief Get the rotation matrix R.
   *
   * @return Eigen::MatrixXd: Rotation matrix
   */
  const Eigen::MatrixXd get_R() const;

  /**
   * @brief Get the position vector p.
   *
   * @return Eigen::VectorXd: Position vector
   */
  const Eigen::MatrixXd get_p() const;

  /**
   * @brief Get the velocity vector v.
   *
   * @return Eigen::VectorXd: Velocity vector
   */
  const Eigen::MatrixXd get_v() const;

  /**
   * @brief Get the dimension of the states matrix X.
   *
   * @return int: Dimension of X
   */
  const int get_K() const;

  /**
   * @brief Get the dimension of the states matrix X.
   *
   * @return int: Dimension of X
   */
  const int get_dim() const;
  /// @}

  ///@name Setters
  /// @{
  /**
   * @brief Set the dimension of matrix X to private variable K_.
   *
   * @param[in] K: Dimension of X
   */
  void set_K(int K);

  /**
   * @brief Set the states matrix X.
   *
   * @param[in] X: States matrix
   */
  void set_X(const Eigen::MatrixXd& X);

  /**
   * @brief Set the rotation matrix R.
   *
   * @param[in] R: Rotation matrix
   */
  void set_R(const Eigen::MatrixXd& R);

  /**
   * @brief Set the position vector p.
   *
   * @param[in] p: Position vector
   */
  void set_p(const Eigen::VectorXd& p);

  /**
   * @brief Set the velocity vector v.
   *
   * @param[in] v: Velocity vector
   */
  void set_v(const Eigen::MatrixXd& v);
  /// @}

  ///@name Overloaded
  /// @{
  /**
   * @brief Overload the multiplication operator.
   *
   * @param[in] X: SEk(3) object
   * @return SEK3: SEk(3) object
   */
  SEK3 operator*(const SEK3& X);
  /// @}

  ///@name Utility functions
  /// @{
  /**
   * @brief Compute the inverse of the SEK3 object.
   *
   * @return SEK3: Inverse of the SEK3 object
   */
  SEK3 inverse();
  /// @}

 private:
  int K_ = 5;
  Eigen::MatrixXd X_ = Eigen::MatrixXd::Identity(K_, K_);

};    // class SEK3

}    // namespace math


#endif    // MATH_SE_3_H