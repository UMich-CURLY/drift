#include <se_k_3.h>

using namespace se_k_3;

// TODO: check size of X
SEK3::SEK3(const Eigen::MatrixXd& X) : X_(X) {}

SEK3::SEK3(const Eigen::MatrixXd& R, const Eigen::VectorXd& p) {
  Eigen::MatrixXd X_ = Eigen::MatrixXd::Identity(4, 4);
  X_.block(0, 0, 3, 3) = R;
  X_.block(0, 3, 3, 1) = p;
  // X_.block(3, 0, 1, 3) = Eigen::Vector3d::Zero();
  // X_.block(3, 3, 1, 1) = Eigen::Vector3d::Ones();
}

SEK3::SEK3(const Eigen::MatrixXd& R, const Eigen::VectorXd& p, const Eigen::VectorXd& v){
  Eigen::MatrixXd X_ = Eigen::MatrixXd::Identity(5, 5);
  X_.block(0, 0, 3, 3) = R;
  X_.block(0, 3, 3, 1) = p;
  X_.block(0, 4, 3, 1) = v;
  // X_.block(3, 0, 2, 4) = Eigen::Vector3d::Zero();
  // X_.block(3, 3, 1, 1) = Eigen::Vector3d::Ones();
  // X_.block(4, 4, 1, 1) = Eigen::Vector3d::Ones();
}
