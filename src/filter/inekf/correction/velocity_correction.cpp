namespace inekf {
using namespace std;
using namespace lie_group;

template<typename sensor_data_t>
VelocityCorrection<sensor_data_t>::VelocityCorrection(
    std::shared_ptr<std::queue<sensor_data_t>> sensor_data_buffer,
    const ErrorType& error_type, const Eigen::Matrix3d& covariance)
    : Correction::Correction(),
      sensor_data_buffer_(sensor_data_buffer),
      error_type_(error_type),
      covariance_(covariance) {}

// Correct using measured body velocity with the estimated velocity
template<typename sensor_data_t>
void VelocityCorrection<sensor_data_t>::Correct(RobotState& state) {
  Eigen::VectorXd Z, Y, b;
  Eigen::MatrixXd H, N, PI;

  // Fill out observation data
  int dimX = state.dimX();
  int dimTheta = state.dimTheta();
  int dimP = state.dimP();

  // Get latest measurement:
  const Eigen::Vector3d measured_velocity = sensor_data_buffer_.get().front();

  // Fill out Y
  // Y.conservativeResize(dimX, Eigen::NoChange);
  // Y.segment(0,dimX) = Eigen::VectorXd::Zero(dimX);
  // Y.segment<3>(0) = measured_velocity;
  // Y(3) = -1;

  // // Fill out b
  // b.conservativeResize(dimX, Eigen::NoChange);
  // b.segment(0,dimX) = Eigen::VectorXd::Zero(dimX);
  // b(3) = -1;

  // Fill out H
  H.conservativeResize(3, dimP);
  H.block(0, 0, 3, dimP) = Eigen::MatrixXd::Zero(3, dimP);
  H.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity();

  // Fill out N
  N.conservativeResize(3, 3);
  N = covariance_;

  // Fill out PI
  // PI.conservativeResize(3, dimX);
  // PI.block(0,0,3,dimX) = Eigen::MatrixXd::Zero(3,dimX);
  // PI.block(0,0,3,3) = Eigen::Matrix3d::Identity();

  // Fill out Z
  // Z = X*Y-b = PI*X*Y
  Eigen::Matrix3d R = state.getRotation();
  Eigen::Vector3d v = state.getVelocity();


  int startIndex = Z.rows();
  Z.conservativeResize(startIndex + 3, Eigen::NoChange);
  Z.segment(0, 3) = R * measured_velocity - v;

  // Correct state using stacked observation
  if (Z.rows() > 0) {
    CorrectRightInvariant(Z, H, N, state, error_type_);
  }
}
}    // namespace inekf