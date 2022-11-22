/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   RobotState.h
 *  @author Ross Hartley
 *  @brief  Source file for RobotState (thread-safe)
 *  @date   September 25, 2018
 **/

#include "lie_group.h"
#include "robot_state.h"

namespace inekf {

using namespace std;

// Default constructor
RobotState::RobotState() : 
    X_(Eigen::MatrixXd::Identity(5,5)), Theta_(Eigen::MatrixXd::Zero(6,1)), P_(Eigen::MatrixXd::Identity(15,15)) {}
// Initialize with X
RobotState::RobotState(const Eigen::MatrixXd& X) : 
    X_(X), Theta_(Eigen::MatrixXd::Zero(6,1)) {
    P_ = Eigen::MatrixXd::Identity(3*this->dimX()+this->dimTheta()-6, 3*this->dimX()+this->dimTheta()-6);
}
// Initialize with X and Theta
RobotState::RobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta) : 
    X_(X), Theta_(Theta) {
    P_ = Eigen::MatrixXd::Identity(3*this->dimX()+this->dimTheta()-6, 3*this->dimX()+this->dimTheta()-6);
}
// Initialize with X, Theta and P
RobotState::RobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta, const Eigen::MatrixXd& P) : 
    X_(X), Theta_(Theta), P_(P) {}

// Initialize with SEK3
RobotState::RobotState(const se_k_3::SEK3& X) : 
    X_(X.getX()), Theta_(Eigen::MatrixXd::Zero(3*(this->dimX()-3),1)) {
    P_ = Eigen::MatrixXd::Identity(3*this->dimX()+this->dimTheta()-6, 3*this->dimX()+this->dimTheta()-6);
}

// Initialize with SEK3 and Theta
RobotState::RobotState(const se_k_3::SEK3& X, const Eigen::VectorXd& Theta) : 
    X_(X.getX()), Theta_(Theta) {
    P_ = Eigen::MatrixXd::Identity(3*this->dimX()+this->dimTheta()-6, 3*this->dimX()+this->dimTheta()-6);
}

// Initialize with SEK3, Theta and P
RobotState::RobotState(const se_k_3::SEK3& X, const Eigen::VectorXd& Theta, const Eigen::MatrixXd& P) : 
    X_(X.getX()), Theta_(Theta), P_(P) {}

// TODO: error checking to make sure dimensions are correct and supported

const Eigen::MatrixXd RobotState::getX() const { return X_; }
const Eigen::VectorXd RobotState::getTheta() const { return Theta_; }
const Eigen::MatrixXd RobotState::getP() const { return P_; }
const Eigen::Matrix3d RobotState::getRotation() const { return X_.block<3,3>(0,0); }
const Eigen::Vector3d RobotState::getVelocity() const { return X_.block<3,1>(0,3); }
const Eigen::Vector3d RobotState::getPosition() const { return X_.block<3,1>(0,4); }
// TODO: protect p1 and v1 incase they are not set
const Eigen::Vector3d RobotState::getp1() const { return X_.block<3,1>(0,5); }
const Eigen::Vector3d RobotState::getv1() const { return X_.block<3,1>(0,6); }
const Eigen::Vector3d RobotState::getVector(int index) const { return X_.block<3,1>(0,index); }

const Eigen::Vector3d RobotState::getGyroscopeBias() const { return Theta_.block<3,1>(0,0); }
const Eigen::Vector3d RobotState::getAccelerometerBias() const { return Theta_.block<3,1>(3,0); }
// TODO: protect p1 and v1 bias incase they are not set
const Eigen::Vector3d RobotState::getp1Bias() const { return Theta_.block<3,1>(4,0); }
const Eigen::Vector3d RobotState::getv1Bias() const { return Theta_.block<3,1>(5,0); }

const Eigen::Matrix3d RobotState::getRotationCovariance() const { return P_.block<3,3>(0,0); }
const Eigen::Matrix3d RobotState::getVelocityCovariance() const { return P_.block<3,3>(3,3); }
const Eigen::Matrix3d RobotState::getPositionCovariance() const { return P_.block<3,3>(6,6); }
const Eigen::Matrix3d RobotState::getGyroscopeBiasCovariance() const { return P_.block<3,3>(9,9); }
const Eigen::Matrix3d RobotState::getAccelerometerBiasCovariance() const { return P_.block<3,3>(12,12); }
// TODO: protect p1 and v1 cov incase they are not set
const Eigen::Matrix3d RobotState::getp1Covariance() const { return P_.block<3,3>(15,15); }
const Eigen::Matrix3d RobotState::getv1Covariance() const { return P_.block<3,3>(18,18); }

const int RobotState::dimX() const { return X_.cols(); }
const int RobotState::dimTheta() const {return Theta_.rows();}
const int RobotState::dimP() const { return P_.cols(); }


const StateType RobotState::getStateType() const { return state_type_; }

const Eigen::MatrixXd RobotState::getWorldX() const {
    if (state_type_ == StateType::WorldCentric) {
        return this->getX();
    } else {
        return this->Xinv();
    }
}
const Eigen::Matrix3d RobotState::getWorldRotation() const {
    if (state_type_ == StateType::WorldCentric) {
        return this->getRotation();
    } else {
        return this->getRotation().transpose();
    }
}
const Eigen::Vector3d RobotState::getWorldVelocity() const {
    if (state_type_ == StateType::WorldCentric) {
        return this->getVelocity();
    } else {
        return -this->getRotation().transpose()*this->getVelocity();
    }
}
const Eigen::Vector3d RobotState::getWorldPosition() const {
    if (state_type_ == StateType::WorldCentric) {
        return this->getPosition();
    } else {
        return -this->getRotation().transpose()*this->getPosition();
    }
}

const Eigen::MatrixXd RobotState::getBodyX() const {
    if (state_type_ == StateType::BodyCentric) {
        return this->getX();
    } else {
        return this->Xinv();
    }
}

const Eigen::Matrix3d RobotState::getBodyRotation() const {
    if (state_type_ == StateType::BodyCentric) {
        return this->getRotation();
    } else {
        return this->getRotation().transpose();
    }
}

const Eigen::Vector3d RobotState::getBodyVelocity() const {
    if (state_type_ == StateType::BodyCentric) {
        return this->getVelocity();
    } else {
        return -this->getRotation().transpose()*this->getVelocity();
    }
}

const Eigen::Vector3d RobotState::getBodyPosition() const {
    if (state_type_ == StateType::BodyCentric) {
        return this->getPosition();
    } else {
        return -this->getRotation().transpose()*this->getPosition();
    }
}


void RobotState::setX(const Eigen::MatrixXd& X) { X_ = X; }
void RobotState::setTheta(const Eigen::VectorXd& Theta) { Theta_ = Theta; }
void RobotState::setP(const Eigen::MatrixXd& P) { P_ = P; }
void RobotState::setRotation(const Eigen::Matrix3d& R) { X_.block<3,3>(0,0) = R; }
void RobotState::setVelocity(const Eigen::Vector3d& v) { X_.block<3,1>(0,3) = v; }
void RobotState::setPosition(const Eigen::Vector3d& p) { X_.block<3,1>(0,4) = p; }

void RobotState::setGyroscopeBias(const Eigen::Vector3d& bg) { Theta_.block<3,1>(0,0) = bg; }
void RobotState::setAccelerometerBias(const Eigen::Vector3d& ba) { Theta_.block<3,1>(0,3) = ba; }
// TODO: protect theta if they don't have v1 and p1 term
void RobotState::setp1(const Eigen::Vector3d& bp1) { X_.block<3,1>(0,6) = bp1; }
void RobotState::setv1(const Eigen::Vector3d& bv1) { X_.block<3,1>(0,9) = bv1; }

void RobotState::setRotationCovariance(const Eigen::Matrix3d& cov) { P_.block<3,3>(0,0) = cov; }
void RobotState::setVelocityCovariance(const Eigen::Matrix3d& cov) { P_.block<3,3>(3,3) = cov; }
void RobotState::setPositionCovariance(const Eigen::Matrix3d& cov) { P_.block<3,3>(6,6) = cov; }
void RobotState::setGyroscopeBiasCovariance(const Eigen::Matrix3d& cov) { P_.block<3,3>(9,9) = cov; }
void RobotState::setAccelerometerBiasCovariance(const Eigen::Matrix3d& cov) { P_.block<3,3>(12,12) = cov; }
// TODO: protect cov if they don't have v1 and p1 term
void RobotState::setp1Covariance(const Eigen::Matrix3d& cov) { P_.block<3,3>(15,15) = cov; }
void RobotState::setv1Covariance(const Eigen::Matrix3d& cov) { P_.block<3,3>(18,18) = cov; }

void RobotState::copyDiagX(int n, Eigen::MatrixXd& BigX) const {
    int dimX = this->dimX();
    for(int i=0; i<n; ++i) {
        int startIndex = BigX.rows();
        BigX.conservativeResize(startIndex + dimX, startIndex + dimX);
        BigX.block(startIndex,0,dimX,startIndex) = Eigen::MatrixXd::Zero(dimX,startIndex);
        BigX.block(0,startIndex,startIndex,dimX) = Eigen::MatrixXd::Zero(startIndex,dimX);
        BigX.block(startIndex,startIndex,dimX,dimX) = X_;
    }
    return;
}

void RobotState::copyDiagXinv(int n, Eigen::MatrixXd& BigXinv) const {
    int dimX = this->dimX();
    Eigen::MatrixXd Xinv = this->Xinv();
    for(int i=0; i<n; ++i) {
        int startIndex = BigXinv.rows();
        BigXinv.conservativeResize(startIndex + dimX, startIndex + dimX);
        BigXinv.block(startIndex,0,dimX,startIndex) = Eigen::MatrixXd::Zero(dimX,startIndex);
        BigXinv.block(0,startIndex,startIndex,dimX) = Eigen::MatrixXd::Zero(startIndex,dimX);
        BigXinv.block(startIndex,startIndex,dimX,dimX) = Xinv;
    }
    return;
}

const Eigen::MatrixXd RobotState::Xinv() const {
    int dimX = this->dimX();
    Eigen::MatrixXd Xinv = Eigen::MatrixXd::Identity(dimX,dimX);
    Eigen::Matrix3d RT = X_.block<3,3>(0,0).transpose();
    Xinv.block<3,3>(0,0) = RT;
    for(int i=3; i<dimX; ++i) {
        Xinv.block<3,1>(0,i) = -RT*X_.block<3,1>(0,i);
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

} // end inekf namespace