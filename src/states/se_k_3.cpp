#include <se_k_3.h>

#include <math.h>
#include <vector>

using namespace se_k_3;

// constructors
// TODO: check size of X
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
Eigen::MatrixXd SEK3::get_X() { return X_; }
Eigen::MatrixXd SEK3::get_R() { return X_.block(0, 0, 3, 3); }
Eigen::MatrixXd SEK3::get_p() { return X_.block(0, 3, 3, 1); }
Eigen::MatrixXd SEK3::get_v() { return X_.block(0, 4, 3, 1); }

Eigen::MatrixXd SEK3::get_p1() { return X_.block(0, 5, 3, 1); }
Eigen::MatrixXd SEK3::get_v1() { return X_.block(0, 6, 3, 1); }
Eigen::MatrixXd SEK3::get_aug(string key) { return X_.block(0, aug_state_map[key]-1, 3, 1); }
vector<string> SEK3::get_aug_keys() { 
  vector<string> keys;
  for (auto const& x : aug_state_map) {
    keys.push_back(x.first);
  }
  return keys;
}
int SEK3::get_aug_val(string key) { return aug_state_map[key]; }
int SEK3::get_dim() { return X_.rows(); }

// setters
void SEK3::set_X(const Eigen::MatrixXd& X) { X_ = X; }
void SEK3::set_R(const Eigen::MatrixXd& R) { X_.block(0, 0, 3, 3) = R; }
void SEK3::set_p(const Eigen::VectorXd& p) { X_.block(0, 3, 3, 1) = p; }
void SEK3::set_v(const Eigen::MatrixXd& v) { X_.block(0, 4, 3, 1) = v; }

// setters - aug state
void SEK3::set_p1(const Eigen::VectorXd& p1) {
  int _K = X_.cols(); + 1
  Eigen::MatrixXd _X = Eigen::MatrixXd::Identity(_K, _K);
  _X.block(0, 0, _K, _K) = X_;
  _X.block(0, _K, 3, 1) = p1;
  X_ = _X;
  K_ = _K + 1;
}

void SEK3::set_v1(const Eigen::VectorXd& v1) {
  int _K = X_.cols() + 1;
  Eigen::MatrixXd _X = Eigen::MatrixXd::Identity(_K, _K);
  _X.block(0, 0, _K, _K) = X_;
  _X.block(0, _K, 3, 1) = v1;
  X_ = _X;
  K_ = _K + 1;
}

void SEK3::set_aug(string key, const Eigen::VectorXd& aug) {
  int _K = X_.cols() + 1;
  aug_state_dict[key] = _K;
  Eigen::MatrixXd _X = Eigen::MatrixXd::Identity(_K, _K);
  _X.block(0, 0, _K, _K) = X_;
  _X.block(0, _K, 3, 1) = aug;
  X_ = _X;
  K_ = _K + 1;
}

// operators
SEK3 SEK3::operator*(const SEK3& X) {
  SEK3 Y;
  // TODO: check size of X, if not same, throw error
  Y.set_X(this->X_* X.X_);
  Y.set_K(this->K_);
  return Y;
}

SEK3 SEK3::operator*(const Eigen::MatrixXd& R) {
  SEK3 Y;
  Y.set_R(this->get_R()* R);
  Y.set_p(this->get_p());
  Y.set_v(this->get_v());
  Y.set_K(this->K_);
  return Y;
}

SEK3 SEK3::operator*(const Eigen::MatrixXd& p) {
  SEK3 Y;
  Y.set_R(this->get_R());
  Y.set_p(this->get_p()+p);
  Y.set_v(this->get_v());
  Y.set_K(this->K_);
  return Y;
}

SEK3 SEK3::operator*(const Eigen::VectorXd& v) {
  SEK3 Y;
  Y.set_R(this->get_R());
  Y.set_p(this->get_p());
  Y.set_v(this->get_v()+v);
  Y.set_K(this->K_);
  return Y;
}

// TODO: / operator
// TODO: << operator

// methods
SEK3 SEK3::inverse() {
  SEK3 Y;
  Y.set_R(this->get_R().transpose());
  Y.set_p(-this->get_R().transpose()*this->get_p());
  Y.set_v(-this->get_R().transpose()*this->get_v());
  Y.set_K(this->K_);
  return Y;
}

SEK3 SEK3::log(){
  SEK3 Y;
  Eigen::MatrixXd R = this->get_R();
  Eigen::VectorXd p = this->get_p();
  Eigen::VectorXd v = this->get_v();
  Eigen::MatrixXd R_log = Eigen::log(R);
  Eigen::VectorXd p_log = R_log*p;
  Eigen::VectorXd v_log = R_log*v;
  Y.set_R(R_log);
  Y.set_p(p_log);
  Y.set_v(v_log);
  Y.set_K(this->K_);
  return Y;
}

SEK3 SEK3::exp(){
  SEK3 Y;
  Eigen::MatrixXd R = this->get_R();
  Eigen::VectorXd p = this->get_p();
  Eigen::VectorXd v = this->get_v();
  Eigen::MatrixXd R_exp = Eigen::exp(R);
  Eigen::VectorXd p_exp = R_exp*p;
  Eigen::VectorXd v_exp = R_exp*v;
  Y.set_R(R_exp);
  Y.set_p(p_exp);
  Y.set_v(v_exp);
  Y.set_K(this->K_);
  return Y;
}
