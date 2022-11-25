/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   Observations.h
 *  @author Ross Hartley
 *  @brief  Header file for Observations
 *  @date   December 03, 2018
 **/

#ifndef INEKF_OBSERVATIONS_H
#define INEKF_OBSERVATIONS_H
#include <Eigen/Dense>
#include <iostream>
#include <map>
#include <vector>

namespace inekf {

// Simple class to hold general observations
class Observation {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // ====================================================================
    /**
     * @brief Constructor. Construct a new Observation object.
     *
     * @param[in] Y: Observation vector
     * @param[in] b: Observation bias vector
     * @param[in] H: Observation Jacobian
     * @param[in] N: Observation noise covariance
     * @param[in] PI: Observation probability of detection
     */
    Observation(Eigen::VectorXd& Y, Eigen::VectorXd& b, Eigen::MatrixXd& H, Eigen::MatrixXd& N, Eigen::MatrixXd& PI);
    bool empty();

    Eigen::VectorXd Y;
    Eigen::VectorXd b;
    Eigen::MatrixXd H;
    Eigen::MatrixXd N;
    Eigen::MatrixXd PI;

    friend std::ostream& operator<<(std::ostream& os, const Observation& o);
};


class Kinematics {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // ====================================================================
    /**
     * @brief Constructor. Construct a new Kinematics object
     *
     * @param[in] id_in: ID of the kinematic measurement
     * @param[in] pose_in: Pose of the kinematic measurement
     * @param[in] covariance_in: Covariance of the kinematic measurement
     */
    Kinematics(int id_in, Eigen::Matrix4d pose_in, Eigen::Matrix<double, 6, 6> covariance_in)
        : id(id_in), pose(pose_in), covariance(covariance_in) {}

    int id;
    Eigen::Matrix4d pose;
    Eigen::Matrix<double, 6, 6> covariance;
};


class Landmark {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // ====================================================================
    /**
     * @brief Constructor. Construct a new Landmark object
     *
     * @param[in] id_in: ID of the landmark measurement
     * @param[in] position_in: Position of the landmark measurement
     * @param[in] covariance_in: Covariance of the landmark measurement
     */
    Landmark(int id_in, Eigen::Vector3d position_in, Eigen::Matrix3d covariance_in)
        : id(id_in), position(position_in), covariance(covariance_in) {}

    int id;
    Eigen::Vector3d position;
    Eigen::Matrix3d covariance;
};


/** A map with an integer as key and a Eigen::Vector3d as value. */
typedef std::map<int, Eigen::Vector3d, std::less<int>,
                 Eigen::aligned_allocator<std::pair<const int, Eigen::Vector3d> > >
    mapIntVector3d;
typedef std::map<int, Eigen::Vector3d, std::less<int>,
                 Eigen::aligned_allocator<std::pair<const int, Eigen::Vector3d> > >::const_iterator
    mapIntVector3dIterator;

/** A vector of Kinematics. */
typedef std::vector<Kinematics, Eigen::aligned_allocator<Kinematics> > vectorKinematics;
typedef std::vector<Kinematics, Eigen::aligned_allocator<Kinematics> >::const_iterator vectorKinematicsIterator;

/** A vector of Landmark. */
typedef std::vector<Landmark, Eigen::aligned_allocator<Landmark> > vectorLandmarks;
typedef std::vector<Landmark, Eigen::aligned_allocator<Landmark> >::const_iterator vectorLandmarksIterator;


}    // namespace inekf
#endif    // end INEKF_OBSERVATIONS_H