/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   lie_group.h
 *  @author Ross Hartley, Tzu-Yuan Lin
 *  @brief  Header file for various Lie Group functions
 *  @date   May 16, 2023
 **/

#ifndef MATH_LIEGROUP_H
#define MATH_LIEGROUP_H

#include <cmath>
#include <iostream>

#include <Eigen/Dense>

namespace math {
namespace lie_group {

extern const double TOLERANCE;

// ======================================================================
/**
 * @brief Computes the factorial of n.
 *
 * @param[in] n: The input integer.
 * @return long int: The factorial result.
 */
long int factorial(int n);

// ======================================================================
/**
 * @brief Convert vector to it's corresponding skew-symmetric matrix.
 * v = [v1,v2,v3]^T.
 * M = [0, -v2, v1,
 *      v2, 0, -v0,
 *      -v1, v0, 0]
 *
 * @param[in] v: Input vector.
 * @return Eigen::Matrix3d: The skew symmetric matrix.
 */
Eigen::Matrix3d skew(const Eigen::Vector3d& v);
// ======================================================================
/**
 * @brief Computes mth integral of the exponential map:
 * \f$ \Gamma_m = \sum_{n=0}^{\infty} \dfrac{1}{(n+m)!} (w^\wedge)^n \f$
 *
 * @param[in] w: Input vector.
 * @param[in] m: mth integral.
 * @return Eigen::Matrix3d: The integral result.
 */
Eigen::Matrix3d Gamma_SO3(const Eigen::Vector3d& w, int m);

// ======================================================================
/**
 * @brief Compute the exponential map for SO(3)
 *
 * @param[in] w: \f$\mathfrak{so(3)}\f$ lie algebra in \f$R^3\f$
 * @return Eigen::Matrix3d: Corresponding SO(3) matrix
 */
Eigen::Matrix3d Exp_SO3(const Eigen::Vector3d& w);

// ======================================================================
/**
 * @brief Compute the left Jacobian of SO(3)
 *
 * @param[in] w: \f$\mathfrak{so(3)}\f$ lie algebra in \f$R^3\f$
 * @return Eigen::Matrix3d: left Jocobian matrix
 */
Eigen::Matrix3d LeftJacobian_SO3(const Eigen::Vector3d& w);

// ======================================================================
/**
 * @brief Compute the right Jacobian of SO(3)
 *
 * @param[in] w: \f$\mathfrak{so(3)}\f$ lie algebra in \f$R^3\f$
 * @return Eigen::Matrix3d: right Jocobian matrix
 */
Eigen::Matrix3d RightJacobian_SO3(const Eigen::Vector3d& w);

// ======================================================================
/**
 * @brief Compute the exponential map for SE_k(3)
 *
 * @param[in] v: \f$\mathfrak{se_k(3)}\f$ lie algebra in \f$R^6\f$, in the order
 * of [omega, v]'
 * @return Eigen::Matrix3d: Corresponding SO(3) matrix
 */
Eigen::MatrixXd Exp_SEK3(const Eigen::VectorXd& v);

// ======================================================================
/**
 * @brief Compute the adjoint map of SE_k(3)
 *
 * @param[in] X: Input SE_k(3) matrix
 * @return Eigen::MatrixXd: The adjoint map
 */
Eigen::MatrixXd Adjoint_SEK3(const Eigen::MatrixXd& X);

}    // namespace lie_group
}    // namespace math
#endif
