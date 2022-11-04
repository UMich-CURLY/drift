/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   robot_state.h
 *  @author Wenzhe Tong
 *  @brief  Header file for RobotState
 *  @date   November 1, 2022
 **/

#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H
#include <iostream>
#include <string>

#include <Eigen/Dense>

#include "math/lie_group.h"
#include "states/se_k_3.h"

namespace inekf {

enum StateType {WorldCentric, BodyCentric};

class RobotState {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        RobotState();
        RobotState(const Eigen::MatrixXd& X);
        RobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta);
        RobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta, const Eigen::MatrixXd& P);
        RobotState(const se_k_3::SEK3& X);
        RobotState(const se_k_3::SEK3& X, const Eigen::VectorXd& Theta);
        RobotState(const se_k_3::SEK3& X, const Eigen::VectorXd& Theta, const Eigen::MatrixXd& P);

        const Eigen::MatrixXd getX() const;
        const Eigen::VectorXd getTheta() const;
        const Eigen::MatrixXd getP() const;
        const Eigen::Matrix3d getRotation() const;
        const Eigen::Vector3d getVelocity() const;
        const Eigen::Vector3d getPosition() const;
        const Eigen::Vector3d getp1() const;
        const Eigen::Vector3d getv1() const;
        const Eigen::Vector3d getVector(int id) const;
        const Eigen::Vector3d getAugState(std::string key) const;

        const Eigen::Vector3d getGyroscopeBias() const;
        const Eigen::Vector3d getAccelerometerBias() const;
        const Eigen::Vector3d getp1Bias() const;
        const Eigen::Vector3d getv1Bias() const;
        const Eigen::Vector3d getAugStateBias(std::string key) const;

        const Eigen::Matrix3d getRotationCovariance() const;
        const Eigen::Matrix3d getVelocityCovariance() const;
        const Eigen::Matrix3d getPositionCovariance() const;
        const Eigen::Matrix3d getGyroscopeBiasCovariance() const;
        const Eigen::Matrix3d getAccelerometerBiasCovariance() const;
        const Eigen::Matrix3d getp1Covariance() const;
        const Eigen::Matrix3d getv1Covariance() const;

        const int dimX() const;
        const int dimTheta() const;
        const int dimP() const;
        const StateType getStateType() const;
        const Eigen::MatrixXd getWorldX() const;
        const Eigen::Matrix3d getWorldRotation() const;
        const Eigen::Vector3d getWorldVelocity() const;
        const Eigen::Vector3d getWorldPosition() const;
        const Eigen::MatrixXd getBodyX() const;
        const Eigen::Matrix3d getBodyRotation() const;
        const Eigen::Vector3d getBodyVelocity() const;
        const Eigen::Vector3d getBodyPosition() const;


        void setX(const Eigen::MatrixXd& X);
        void setP(const Eigen::MatrixXd& P);
        void setTheta(const Eigen::VectorXd& Theta);
        void setRotation(const Eigen::Matrix3d& R);
        void setVelocity(const Eigen::Vector3d& v);
        void setPosition(const Eigen::Vector3d& p);
        void setGyroscopeBias(const Eigen::Vector3d& bg);
        void setAccelerometerBias(const Eigen::Vector3d& ba);
        void setRotationCovariance(const Eigen::Matrix3d& cov);
        void setVelocityCovariance(const Eigen::Matrix3d& cov);
        void setPositionCovariance(const Eigen::Matrix3d& cov);
        void setGyroscopeBiasCovariance(const Eigen::Matrix3d& cov);
        void setAccelerometerBiasCovariance(const Eigen::Matrix3d& cov);
        void copyDiagX(int n, Eigen::MatrixXd& BigX) const;
        void copyDiagXinv(int n, Eigen::MatrixXd& BigXinv) const;

        const Eigen::MatrixXd Xinv() const;

        friend std::ostream& operator<<(std::ostream& os, const RobotState& s);  

    private:
        StateType state_type_ = StateType::WorldCentric; 
        Eigen::MatrixXd X_;
        Eigen::VectorXd Theta_;
        Eigen::MatrixXd P_;
};

} // end inekf namespace
#endif 
