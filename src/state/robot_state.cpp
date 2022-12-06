/**
 *  @file   robot_state.cpp
 *  @author Ross Hartley, Wenzhe Tong
 *  @brief  Source file for RobotState
 *  @date   Nov 1st, 2022
 **/

#include "state/robot_state.h"

using namespace std;

// Default constructor
RobotState::RobotState()
    : X_(Eigen::MatrixXd::Identity(5, 5)),
      Theta_(Eigen::MatrixXd::Zero(6, 1)),
      P_(Eigen::MatrixXd::Identity(15, 15)) {}

// Initialize with X
RobotState::RobotState(const Eigen::MatrixXd& X)
    : X_(X), Theta_(Eigen::MatrixXd::Zero(6, 1)) {
  P_ = Eigen::MatrixXd::Identity(3 * this->dimX() + this->dimTheta() - 6,
                                 3 * this->dimX() + this->dimTheta() - 6);
}

// Initialize with X and Theta
RobotState::RobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta)
    : X_(X), Theta_(Theta) {
  P_ = Eigen::MatrixXd::Identity(3 * this->dimX() + this->dimTheta() - 6,
                                 3 * this->dimX() + this->dimTheta() - 6);
}

// Initialize with X, Theta and P
RobotState::RobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta,
                       const Eigen::MatrixXd& P)
    : X_(X), Theta_(Theta), P_(P) {}

// Initialize with SEK3
RobotState::RobotState(SEK3& X)
    : X_(X.get_X()), Theta_(Eigen::MatrixXd::Zero(3 * (this->dimX() - 3), 1)) {
  P_ = Eigen::MatrixXd::Identity(3 * this->dimX() + this->dimTheta() - 6,
                                 3 * this->dimX() + this->dimTheta() - 6);
}

// Initialize with SEK3 and Theta
RobotState::RobotState(SEK3& X, const Eigen::VectorXd& Theta)
    : X_(X.get_X()), Theta_(Theta) {
  P_ = Eigen::MatrixXd::Identity(3 * this->dimX() + this->dimTheta() - 6,
                                 3 * this->dimX() + this->dimTheta() - 6);
}

// Initialize with SEK3, Theta and P
RobotState::RobotState(SEK3& X, const Eigen::VectorXd& Theta,
                       const Eigen::MatrixXd& P)
    : X_(X.get_X()), Theta_(Theta), P_(P) {}

// getters
const Eigen::MatrixXd RobotState::getX() const { return X_; }
const Eigen::VectorXd RobotState::getTheta() const { return Theta_; }
const Eigen::MatrixXd RobotState::getP() const { return P_; }
const Eigen::Matrix3d RobotState::getRotation() const {
  return X_.block<3, 3>(0, 0);
}
const Eigen::Vector3d RobotState::getVelocity() const {
  return X_.block<3, 1>(0, 3);
}
const Eigen::Vector3d RobotState::getPosition() const {
  return X_.block<3, 1>(0, 4);
}
const Eigen::Vector3d RobotState::getVector(int index) const {
  return X_.block<3, 1>(0, index);
}
// const Eigen::Vector3d RobotState::getAugState(std::string key) {
//     int idx = SEK3::get_aug_index(key) - 1;
//     return X_.block<3,1>(0,idx);
// }

const Eigen::Vector3d RobotState::getGyroscopeBias() const {
  return Theta_.block<3, 1>(0, 0);
}
const Eigen::Vector3d RobotState::getAccelerometerBias() const {
  return Theta_.block<3, 1>(3, 0);
}
// const Eigen::Vector3d RobotState::getAugStateBias(std::string key) {
//     int idx = SEK3::get_aug_index(key) - 1;
//     return Theta_.block<3,1>(idx,0);
// }

const Eigen::Matrix3d RobotState::getRotationCovariance() const {
  return P_.block<3, 3>(0, 0);
}
const Eigen::Matrix3d RobotState::getVelocityCovariance() const {
  return P_.block<3, 3>(3, 3);
}
const Eigen::Matrix3d RobotState::getPositionCovariance() const {
  return P_.block<3, 3>(6, 6);
}
const Eigen::Matrix3d RobotState::getGyroscopeBiasCovariance() const {
  return P_.block<3, 3>(9, 9);
}
const Eigen::Matrix3d RobotState::getAccelerometerBiasCovariance() const {
  return P_.block<3, 3>(12, 12);
}

std::vector<std::map<int, int>> RobotState::get_augmented_maps() {
  return idx_maps_;
}

int RobotState::add_augmented_map() {
  std::map<int, int> idx_map;
  idx_maps_.push_back(idx_map);
  return idx_maps_.size() - 1;
}

std::map<int, int> RobotState::get_augmented_map(int idx) {
  return idx_maps_[idx];
}

int RobotState::add_aug_state(int idx_map, const Eigen::Vector3d& aug) {
  if (idx_map < 0 || idx_map >= idx_maps_.size()) {
    throw std::invalid_argument("Invalid map index");
  }
  if (aug.size() != 3) {
    throw std::invalid_argument("Invalid augmented state size");
  }
  Eigen::MatrixXd X_aug
      = Eigen::MatrixXd::Identity(this->dimX() + 1, this->dimX() + 1);
  X_aug.block(0, 0, this->dimX(), this->dimX()) = X_;
  X_aug.block(0, this->dimX(), 3, 1) = aug;
  X_ = X_aug;
  // TODO: check this->dimX, it is 1 based, check map[][]
  int idx_state = idx_maps_[idx_map].size();
  idx_maps_[idx_map].insert(std::pair<int, int>(idx_state, this->dimX()));
}

void RobotState::set_aug_state(int idx_map, int idx_state,
                               const Eigen::Vector3d& aug) {
  if (idx_state < 0 || idx_state >= this->dimX()) {
    throw std::invalid_argument("Invalid state index");
  }
  if (idx_map < 0 || idx_map >= idx_maps_.size()) {
    throw std::invalid_argument("Invalid map index");
  }
  if (aug.size() != 3) {
    throw std::invalid_argument("Invalid augmented state size");
  }
  if (idx_maps_[idx_map].find(idx_state) == idx_maps_[idx_map].end()) {
    throw std::invalid_argument("Augmented state does not exist");
  }
  X_.block<3, 1>(0, idx_maps_[idx_map][idx_state]) = aug;
}

void RobotState::del_aug_state(int idx_map, int idx_state) {
  if (idx_state < 0 || idx_state >= this->dimX()) {
    throw std::invalid_argument("Invalid state index");
  }
  if (idx_map < 0 || idx_map >= idx_maps_.size()) {
    throw std::invalid_argument("Invalid map index");
  }
  if (idx_maps_[idx_map].find(idx_state) == idx_maps_[idx_map].end()) {
    throw std::invalid_argument("Augmented state does not exist");
  }
  Eigen::MatrixXd X_aug
      = Eigen::MatrixXd::Identity(this->dimX() - 1, this->dimX() - 1);
  X_aug.block(0, 0, idx_maps_[idx_map][idx_state],
              idx_maps_[idx_map][idx_state])
      = X_.block(0, 0, idx_maps_[idx_map][idx_state],
                 idx_maps_[idx_map][idx_state]);
  X_aug.block(idx_maps_[idx_map][idx_state], idx_maps_[idx_map][idx_state],
              this->dimX() - idx_maps_[idx_map][idx_state] - 1,
              this->dimX() - idx_maps_[idx_map][idx_state] - 1)
      = X_.block(idx_maps_[idx_map][idx_state] + 1,
                 idx_maps_[idx_map][idx_state] + 1,
                 this->dimX() - idx_maps_[idx_map][idx_state] - 1,
                 this->dimX() - idx_maps_[idx_map][idx_state] - 1);
  X_ = X_aug;
  idx_maps_[idx_map].erase(idx_state);
}

const Eigen::Vector3d RobotState::get_aug_state(int idx_map, int idx_state) {
  if (idx_state < 0 || idx_state >= this->dimX()) {
    throw std::invalid_argument("Invalid state index");
  }
  if (idx_map < 0 || idx_map >= this->idx_maps_.size()) {
    throw std::invalid_argument("Invalid map index");
  }
  if (idx_maps_[idx_map].find(idx_state) == idx_maps_[idx_map].end()) {
    throw std::invalid_argument("Augmented state does not exist");
  }
  return X_.block<3, 1>(0, idx_maps_[idx_map][idx_state]);
}

const int RobotState::dimX() const { return X_.cols(); }
const int RobotState::dimTheta() const { return Theta_.rows(); }
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
    return -this->getRotation().transpose() * this->getVelocity();
  }
}

const Eigen::Vector3d RobotState::getWorldPosition() const {
  if (state_type_ == StateType::WorldCentric) {
    return this->getPosition();
  } else {
    return -this->getRotation().transpose() * this->getPosition();
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
    return -this->getRotation().transpose() * this->getVelocity();
  }
}

const Eigen::Vector3d RobotState::getBodyPosition() const {
  if (state_type_ == StateType::BodyCentric) {
    return this->getPosition();
  } else {
    return -this->getRotation().transpose() * this->getPosition();
  }
}

// setters
void RobotState::setX(const Eigen::MatrixXd& X) { X_ = X; }
void RobotState::setTheta(const Eigen::VectorXd& Theta) { Theta_ = Theta; }
void RobotState::setP(const Eigen::MatrixXd& P) { P_ = P; }
void RobotState::setRotation(const Eigen::Matrix3d& R) {
  X_.block<3, 3>(0, 0) = R;
}
void RobotState::setVelocity(const Eigen::Vector3d& v) {
  X_.block<3, 1>(0, 3) = v;
}
void RobotState::setPosition(const Eigen::Vector3d& p) {
  X_.block<3, 1>(0, 4) = p;
}

void RobotState::setGyroscopeBias(const Eigen::Vector3d& bg) {
  Theta_.block<3, 1>(0, 0) = bg;
}
void RobotState::setAccelerometerBias(const Eigen::Vector3d& ba) {
  Theta_.block<3, 1>(0, 3) = ba;
}

int RobotState::add_aug_bias(int idx_map, const Eigen::Vector3d& baug) {
  if (idx_map < 0 || idx_map >= idx_maps_.size()) {
    throw std::invalid_argument("Invalid map index");
  }
  if (baug.size() != 3) {
    throw std::invalid_argument("Invalid bias size");
  }
  int dim = this->dimTheta();
  Eigen::VectorXd Theta_aug = Eigen::VectorXd::Zero(dim + 3);
  Theta_aug.block(0, 0, 1, dim) = Theta_;
  Theta_aug.block<3, 1>(0, dim) = baug;
  Theta_ = Theta_aug;
  return dim;
}

void RobotState::set_aug_bias(int idx_map, int idx_state,
                              const Eigen::Vector3d& baug) {
  if (idx_state < 0 || idx_state >= this->dimTheta()) {
    throw std::invalid_argument("Invalid state index");
  }
  if (idx_map < 0 || idx_map >= idx_maps_.size()) {
    throw std::invalid_argument("Invalid map index");
  }
  if (idx_maps_[idx_map].find(idx_state) == idx_maps_[idx_map].end()) {
    throw std::invalid_argument("Augmented state does not exist");
  }
  Theta_.block<3, 1>(0, idx_maps_[idx_map][idx_state] * 3 - 12) = baug;
}

void RobotState::del_aug_bias(int idx_map, int idx_state) {
  if (idx_state < 0 || idx_state >= this->dimTheta()) {
    throw std::invalid_argument("Invalid state index");
  }
  if (idx_map < 0 || idx_map >= idx_maps_.size()) {
    throw std::invalid_argument("Invalid map index");
  }
  if (idx_maps_[idx_map].find(idx_state) == idx_maps_[idx_map].end()) {
    throw std::invalid_argument("Augmented state does not exist");
  }
  int dim = this->dimTheta();
  Eigen::VectorXd Theta_aug = Eigen::VectorXd::Zero(dim - 3);
  int idx_bias = idx_maps_[idx_map][idx_state] * 3 - 12;
  Theta_aug.block(0, 0, 1, idx_bias) = Theta_.block(0, 0, 1, idx_bias);
  Theta_aug.block(0, idx_bias, 1, dim - idx_bias - 3)
      = Theta_.block(0, idx_bias + 3, 1, dim - idx_bias - 3);
  Theta_ = Theta_aug;
  idx_maps_[idx_map].erase(idx_state);
}

const Eigen::Vector3d RobotState::get_aug_bias(int idx_map, int idx_state) {
  if (idx_state < 0 || idx_state >= this->dimTheta()) {
    throw std::invalid_argument("Invalid state index");
  }
  if (idx_map < 0 || idx_map >= idx_maps_.size()) {
    throw std::invalid_argument("Invalid map index");
  }
  if (idx_maps_[idx_map].find(idx_state) == idx_maps_[idx_map].end()) {
    throw std::invalid_argument("Augmented state does not exist");
  }
  int idx_bias = idx_maps_[idx_map][idx_state] * 3 - 12;
  return Theta_.block<3, 1>(0, idx_bias);
}

void RobotState::setRotationCovariance(const Eigen::Matrix3d& cov) {
  P_.block<3, 3>(0, 0) = cov;
}
void RobotState::setVelocityCovariance(const Eigen::Matrix3d& cov) {
  P_.block<3, 3>(3, 3) = cov;
}
void RobotState::setPositionCovariance(const Eigen::Matrix3d& cov) {
  P_.block<3, 3>(6, 6) = cov;
}
void RobotState::setGyroscopeBiasCovariance(const Eigen::Matrix3d& cov) {
  P_.block<3, 3>(9, 9) = cov;
}
void RobotState::setAccelerometerBiasCovariance(const Eigen::Matrix3d& cov) {
  P_.block<3, 3>(12, 12) = cov;
}
// void setAugStateCovariance(std::string key, const Eigen::Matrix3d& cov) {
//     int idx = SEK3::get_aug_index(key) - 1;
//     P_.block<3,3>(3*idx,3*idx) = cov;
// }

int RobotState::add_aug_cov(int idx_map, const Eigen::Matrix3d& cov) {
  if (idx_map < 0 || idx_map >= idx_maps_.size()) {
    throw std::invalid_argument("Invalid map index");
  }
  int dim = this->dimP();
  Eigen::MatrixXd P_aug = Eigen::MatrixXd::Zero(dim + 3, dim + 3);
  P_aug.block(0, 0, dim, dim) = P_;
  P_aug.block<3, 3>(dim, dim) = cov;
  P_ = P_aug;
  return dim;
}

void RobotState::set_aug_cov(int idx_map, int idx_state,
                             const Eigen::Matrix3d& cov) {
  if (idx_state < 0 || idx_state >= this->dimP()) {
    throw std::invalid_argument("Invalid state index");
  }
  if (idx_map < 0 || idx_map >= idx_maps_.size()) {
    throw std::invalid_argument("Invalid map index");
  }
  if (idx_maps_[idx_map].find(idx_state) == idx_maps_[idx_map].end()) {
    throw std::invalid_argument("Augmented state does not exist");
  }
  P_.block<3, 3>(0, idx_maps_[idx_map][idx_state] * 3 - 12) = cov;
}

void RobotState::del_aug_cov(int idx_map, int idx_state) {
  if (idx_state < 0 || idx_state >= this->dimP()) {
    throw std::invalid_argument("Invalid state index");
  }
  if (idx_map < 0 || idx_map >= idx_maps_.size()) {
    throw std::invalid_argument("Invalid map index");
  }
  if (idx_maps_[idx_map].find(idx_state) == idx_maps_[idx_map].end()) {
    throw std::invalid_argument("Augmented state does not exist");
  }
  int dim = this->dimP();
  Eigen::MatrixXd P_aug = Eigen::MatrixXd::Zero(dim - 3, dim - 3);
  int idx_cov = idx_maps_[idx_map][idx_state] * 3 - 12;
  P_aug.block(0, 0, idx_cov, idx_cov) = P_.block(0, 0, idx_cov, idx_cov);
  P_aug.block(idx_cov, idx_cov, dim - idx_cov - 3, dim - idx_cov - 3)
      = P_.block(idx_cov + 3, idx_cov + 3, dim - idx_cov - 3,
                 dim - idx_cov - 3);
  P_ = P_aug;
  idx_maps_[idx_map].erase(idx_state);
}

const Eigen::Matrix3d RobotState::get_aug_cov(int idx_map, int idx_state) {
  if (idx_state < 0 || idx_state >= this->dimP()) {
    throw std::invalid_argument("Invalid state index");
  }
  if (idx_map < 0 || idx_map >= idx_maps_.size()) {
    throw std::invalid_argument("Invalid map index");
  }
  if (idx_maps_[idx_map].find(idx_state) == idx_maps_[idx_map].end()) {
    throw std::invalid_argument("Augmented state does not exist");
  }
  int idx_cov = idx_maps_[idx_map][idx_state] * 3 - 12;
  return P_.block<3, 3>(idx_cov, idx_cov);
}

void RobotState::copyDiagX(int n, Eigen::MatrixXd& BigX) const {
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

void RobotState::copyDiagXinv(int n, Eigen::MatrixXd& BigXinv) const {
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
