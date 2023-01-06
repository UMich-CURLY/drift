#include "filter/inekf/propagation/imu_propagation.h"

namespace inekf {
using namespace std;
using namespace lie_group;

// IMU propagation child class
// ==============================================================================
// IMU propagation constructor
ImuPropagation::ImuPropagation(
    IMUQueuePtr sensor_data_buffer_ptr,
    std::shared_ptr<std::mutex> sensor_data_buffer_mutex_ptr,
    const NoiseParams& params, const ErrorType& error_type,
    const std::string& yaml_filepath)
    : Propagation::Propagation(params, sensor_data_buffer_mutex_ptr),
      sensor_data_buffer_ptr_(sensor_data_buffer_ptr),
      error_type_(error_type) {
  propagation_type_ = PropagationType::IMU;

  cout << "Loading imu propagation config from " << yaml_filepath << endl;
  YAML::Node config_ = YAML::LoadFile(yaml_filepath);

  // Setting config parameters
  static_bias_initialization_
      = config_["settings"]["static_bias_initialization"]
            ? config_["settings"]["static_bias_initialization"].as<bool>()
            : true;

  estimate_bias_ = config_["settings"]["enable_bias_update"]
                       ? config_["settings"]["enable_bias_update"].as<bool>()
                       : true;

  init_bias_size_ = config_["settings"]["init_bias_size"]
                        ? config_["settings"]["init_bias_size"].as<int>()
                        : 250;
  use_imu_ori_est_init_bias_
      = config_["settings"]["use_imu_ori_est_init_bias"]
            ? config_["settings"]["use_imu_ori_est_init_bias"].as<bool>()
            : false;

  // Set the imu to body rotation
  const std::vector<double> quat_imu2body
      = config_["settings"]["rotation_imu2body"]
            ? config_["settings"]["rotation_imu2body"].as<std::vector<double>>()
            : std::vector<double>({1, 0, 0, 0});
  R_imu2body_ = compute_R_imu2body(quat_imu2body);

  // Set the noise parameters
  double gyro_std = config_["noises"]["gyroscope_std"]
                        ? config_["noises"]["gyroscope_std"].as<double>()
                        : 0.1;
  double accel_std = config_["noises"]["accelerometer_std"]
                         ? config_["noises"]["accelerometer_std"].as<double>()
                         : 0.1;
  double gyro_bias_std
      = config_["noises"]["gyroscope_bias_std"]
            ? config_["noises"]["gyroscope_bias_std"].as<double>()
            : 0.1;
  double accel_bias_std
      = config_["noises"]["accelerometer_bias_std"]
            ? config_["noises"]["accelerometer_bias_std"].as<double>()
            : 0.1;

  gyro_cov_ = gyro_std * gyro_std * Eigen::Matrix<double, 3, 3>::Identity();
  gyro_bias_cov_
      = gyro_bias_std * gyro_bias_std * Eigen::Matrix<double, 3, 3>::Identity();
  accel_cov_ = accel_std * accel_std * Eigen::Matrix<double, 3, 3>::Identity();
  accel_bias_cov_ = accel_bias_std * accel_bias_std
                    * Eigen::Matrix<double, 3, 3>::Identity();

  // Set the initial bias
  std::vector<double> gyroscope_bias
      = config_["priors"]["gyroscope_bias"]
            ? config_["priors"]["gyroscope_bias"].as<std::vector<double>>()
            : std::vector<double>({0, 0, 0});
  bg0_ = Eigen::Vector3d(gyroscope_bias[0], gyroscope_bias[1],
                         gyroscope_bias[2]);

  std::vector<double> accelerometer_bias
      = config_["priors"]["accelerometer_bias"]
            ? config_["priors"]["accelerometer_bias"].as<std::vector<double>>()
            : std::vector<double>({0, 0, 0});
  ba0_ = Eigen::Vector3d(accelerometer_bias[0], accelerometer_bias[1],
                         accelerometer_bias[2]);
}

const IMUQueuePtr ImuPropagation::get_sensor_data_buffer_ptr() const {
  return sensor_data_buffer_ptr_;
}

// IMU propagation method
bool ImuPropagation::Propagate(RobotState& state) {
  // Bias corrected IMU measurements

  /// TODO: double check :
  sensor_data_buffer_mutex_ptr_.get()->lock();
  if (sensor_data_buffer_ptr_->empty()) {
    sensor_data_buffer_mutex_ptr_.get()->unlock();
    return false;
  }
  auto imu_measurement = *(sensor_data_buffer_ptr_->front().get());
  sensor_data_buffer_ptr_->pop();
  sensor_data_buffer_mutex_ptr_.get()->unlock();

  double dt = imu_measurement.get_time() - state.get_propagate_time();
  state.set_time(imu_measurement.get_time());
  state.set_propagate_time(imu_measurement.get_time());

  Eigen::Vector3d w = imu_measurement.get_ang_vel()
                      - state.get_gyroscope_bias();    // Angular Velocity

  Eigen::Vector3d a
      = imu_measurement.get_lin_acc()
        - state.get_accelerometer_bias();    // Linear Acceleration

  // Rotate imu frame to align it with the body frame:
  w = R_imu2body_ * w;
  a = R_imu2body_ * a;

  // Get current state estimate and dimensions
  Eigen::MatrixXd X = state.get_X();
  Eigen::MatrixXd Xinv = state.Xinv();
  Eigen::MatrixXd P = state.get_P();
  int dimX = state.dimX();
  int dimP = state.dimP();
  int dimTheta = state.dimTheta();

  //  ------------ Propagate Covariance --------------- //
  Eigen::MatrixXd Phi = this->StateTransitionMatrix(w, a, dt, state);
  Eigen::MatrixXd Qd = this->DiscreteNoiseMatrix(Phi, dt, state);
  Eigen::MatrixXd P_pred = Phi * P * Phi.transpose() + Qd;

  // If we don't want to estimate bias, remove correlation
  if (!estimate_bias_) {
    P_pred.block(0, dimP - dimTheta, dimP - dimTheta, dimTheta)
        = Eigen::MatrixXd::Zero(dimP - dimTheta, dimTheta);
    P_pred.block(dimP - dimTheta, 0, dimTheta, dimP - dimTheta)
        = Eigen::MatrixXd::Zero(dimTheta, dimP - dimTheta);
    P_pred.block(dimP - dimTheta, dimP - dimTheta, dimTheta, dimTheta)
        = Eigen::MatrixXd::Identity(dimTheta, dimTheta);
  }

  //  ------------ Propagate Mean --------------- //
  Eigen::Matrix3d R = state.get_rotation();
  Eigen::Vector3d v = state.get_velocity();
  Eigen::Vector3d p = state.get_position();

  Eigen::Vector3d phi = w * dt;
  Eigen::Matrix3d G0 = Gamma_SO3(
      phi,
      0);    // Computation can be sped up by computing G0,G1,G2 all at once
  Eigen::Matrix3d G1 = Gamma_SO3(phi, 1);
  Eigen::Matrix3d G2 = Gamma_SO3(phi, 2);

  Eigen::MatrixXd X_pred = X;
  if (state.get_state_type() == StateType::WorldCentric) {
    // Propagate world-centric state estimate
    X_pred.block<3, 3>(0, 0) = R * G0;
    X_pred.block<3, 1>(0, 3) = v + (R * G1 * a + g_) * dt;
    X_pred.block<3, 1>(0, 4) = p + v * dt + (R * G2 * a + 0.5 * g_) * dt * dt;
  } else {
    // Propagate body-centric state estimate
    Eigen::MatrixXd X_pred = X;
    Eigen::Matrix3d G0t = G0.transpose();
    X_pred.block<3, 3>(0, 0) = G0t * R;
    X_pred.block<3, 1>(0, 3) = G0t * (v - (G1 * a + R * g_) * dt);
    X_pred.block<3, 1>(0, 4)
        = G0t * (p + v * dt - (G2 * a + 0.5 * R * g_) * dt * dt);
    for (int i = 5; i < dimX; ++i) {
      X_pred.block<3, 1>(0, i) = G0t * X.block<3, 1>(0, i);
    }
  }

  //  ------------ Update State --------------- //
  state.set_X(X_pred);
  state.set_P(P_pred);

  return true;
}

// Compute Analytical state transition matrix
Eigen::MatrixXd ImuPropagation::StateTransitionMatrix(const Eigen::Vector3d& w,
                                                      const Eigen::Vector3d& a,
                                                      const double dt,
                                                      const RobotState& state) {
  Eigen::Vector3d phi = w * dt;
  Eigen::Matrix3d G0 = Gamma_SO3(
      phi,
      0);    // Computation can be sped up by computing G0,G1,G2 all at once
  Eigen::Matrix3d G1
      = Gamma_SO3(phi, 1);    // TODO: These are also needed for the mean
                              // propagation, we should not compute twice
  Eigen::Matrix3d G2 = Gamma_SO3(phi, 2);
  Eigen::Matrix3d G0t = G0.transpose();
  Eigen::Matrix3d G1t = G1.transpose();
  Eigen::Matrix3d G2t = G2.transpose();
  Eigen::Matrix3d G3t = Gamma_SO3(-phi, 3);

  int dimX = state.dimX();
  int dimTheta = state.dimTheta();
  int dimP = state.dimP();
  Eigen::MatrixXd Phi = Eigen::MatrixXd::Identity(dimP, dimP);

  // Compute the complicated bias terms (derived for the left invariant case)
  Eigen::Matrix3d ax = skew(a);
  Eigen::Matrix3d wx = skew(w);
  Eigen::Matrix3d wx2 = wx * wx;
  double dt2 = dt * dt;
  double dt3 = dt2 * dt;
  double theta = w.norm();
  double theta2 = theta * theta;
  double theta3 = theta2 * theta;
  double theta4 = theta3 * theta;
  double theta5 = theta4 * theta;
  double theta6 = theta5 * theta;
  double theta7 = theta6 * theta;
  double thetadt = theta * dt;
  double thetadt2 = thetadt * thetadt;
  double thetadt3 = thetadt2 * thetadt;
  double sinthetadt = sin(thetadt);
  double costhetadt = cos(thetadt);
  double sin2thetadt = sin(2 * thetadt);
  double cos2thetadt = cos(2 * thetadt);
  double thetadtcosthetadt = thetadt * costhetadt;
  double thetadtsinthetadt = thetadt * sinthetadt;

  Eigen::Matrix3d Phi25L
      = G0t
        * (ax * G2t * dt2
           + ((sinthetadt - thetadtcosthetadt) / (theta3)) * (wx * ax)
           - ((cos2thetadt - 4 * costhetadt + 3) / (4 * theta4))
                 * (wx * ax * wx)
           + ((4 * sinthetadt + sin2thetadt - 4 * thetadtcosthetadt
               - 2 * thetadt)
              / (4 * theta5))
                 * (wx * ax * wx2)
           + ((thetadt2 - 2 * thetadtsinthetadt - 2 * costhetadt + 2)
              / (2 * theta4))
                 * (wx2 * ax)
           - ((6 * thetadt - 8 * sinthetadt + sin2thetadt) / (4 * theta5))
                 * (wx2 * ax * wx)
           + ((2 * thetadt2 - 4 * thetadtsinthetadt - cos2thetadt + 1)
              / (4 * theta6))
                 * (wx2 * ax * wx2));

  Eigen::Matrix3d Phi35L
      = G0t
        * (ax * G3t * dt3
           - ((thetadtsinthetadt + 2 * costhetadt - 2) / (theta4)) * (wx * ax)
           - ((6 * thetadt - 8 * sinthetadt + sin2thetadt) / (8 * theta5))
                 * (wx * ax * wx)
           - ((2 * thetadt2 + 8 * thetadtsinthetadt + 16 * costhetadt
               + cos2thetadt - 17)
              / (8 * theta6))
                 * (wx * ax * wx2)
           + ((thetadt3 + 6 * thetadt - 12 * sinthetadt + 6 * thetadtcosthetadt)
              / (6 * theta5))
                 * (wx2 * ax)
           - ((6 * thetadt2 + 16 * costhetadt - cos2thetadt - 15)
              / (8 * theta6))
                 * (wx2 * ax * wx)
           + ((4 * thetadt3 + 6 * thetadt - 24 * sinthetadt - 3 * sin2thetadt
               + 24 * thetadtcosthetadt)
              / (24 * theta7))
                 * (wx2 * ax * wx2));


  // TODO: Get better approximation using taylor series when theta < tol
  const double tol = 1e-6;
  if (theta < tol) {
    Phi25L = (1 / 2) * ax * dt2;
    Phi35L = (1 / 6) * ax * dt3;
  }

  // Fill out analytical state transition matrices
  if ((state.get_state_type() == StateType::WorldCentric
       && error_type_ == ErrorType::LeftInvariant)
      || (state.get_state_type() == StateType::BodyCentric
          && error_type_ == ErrorType::RightInvariant)) {
    // Compute left-invariant state transisition matrix
    Phi.block<3, 3>(0, 0) = G0t;                          // Phi_11
    Phi.block<3, 3>(3, 0) = -G0t * skew(G1 * a) * dt;     // Phi_21
    Phi.block<3, 3>(6, 0) = -G0t * skew(G2 * a) * dt2;    // Phi_31
    Phi.block<3, 3>(3, 3) = G0t;                          // Phi_22
    Phi.block<3, 3>(6, 3) = G0t * dt;                     // Phi_32
    Phi.block<3, 3>(6, 6) = G0t;                          // Phi_33
    for (int i = 5; i < dimX; ++i) {
      Phi.block<3, 3>((i - 2) * 3, (i - 2) * 3) = G0t;    // Phi_(3+i)(3+i)
    }
    Phi.block<3, 3>(0, dimP - dimTheta) = -G1t * dt;              // Phi_15
    Phi.block<3, 3>(3, dimP - dimTheta) = Phi25L;                 // Phi_25
    Phi.block<3, 3>(6, dimP - dimTheta) = Phi35L;                 // Phi_35
    Phi.block<3, 3>(3, dimP - dimTheta + 3) = -G1t * dt;          // Phi_26
    Phi.block<3, 3>(6, dimP - dimTheta + 3) = -G0t * G2 * dt2;    // Phi_36
  } else {
    // Compute right-invariant state transition matrix (Assumes unpropagated
    // state)
    Eigen::Matrix3d gx = skew(g_);
    Eigen::Matrix3d R = state.get_rotation();
    Eigen::Vector3d v = state.get_velocity();
    Eigen::Vector3d p = state.get_position();
    Eigen::Matrix3d RG0 = R * G0;
    Eigen::Matrix3d RG1dt = R * G1 * dt;
    Eigen::Matrix3d RG2dt2 = R * G2 * dt2;
    Phi.block<3, 3>(3, 0) = gx * dt;                             // Phi_21
    Phi.block<3, 3>(6, 0) = 0.5 * gx * dt2;                      // Phi_31
    Phi.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity() * dt;    // Phi_32
    Phi.block<3, 3>(0, dimP - dimTheta) = -RG1dt;                // Phi_15
    Phi.block<3, 3>(3, dimP - dimTheta)
        = -skew(v + RG1dt * a + g_ * dt) * RG1dt + RG0 * Phi25L;    // Phi_25
    Phi.block<3, 3>(6, dimP - dimTheta)
        = -skew(p + v * dt + RG2dt2 * a + 0.5 * g_ * dt2) * RG1dt
          + RG0 * Phi35L;    // Phi_35
    for (int i = 5; i < dimX; ++i) {
      Phi.block<3, 3>((i - 2) * 3, dimP - dimTheta)
          = -skew(state.get_vector(i)) * RG1dt;    // Phi_(3+i)5
    }
    Phi.block<3, 3>(3, dimP - dimTheta + 3) = -RG1dt;     // Phi_26
    Phi.block<3, 3>(6, dimP - dimTheta + 3) = -RG2dt2;    // Phi_36
  }
  return Phi;
}

// Compute Discrete noise matrix
Eigen::MatrixXd ImuPropagation::DiscreteNoiseMatrix(const Eigen::MatrixXd& Phi,
                                                    const double dt,
                                                    const RobotState& state) {
  int dimX = state.dimX();
  int dimTheta = state.dimTheta();
  int dimP = state.dimP();
  Eigen::MatrixXd G = Eigen::MatrixXd::Identity(dimP, dimP);

  // Compute G using Adjoint of Xk if needed, otherwise identity (Assumes
  // unpropagated state)
  if ((state.get_state_type() == StateType::WorldCentric
       && error_type_ == ErrorType::RightInvariant)
      || (state.get_state_type() == StateType::BodyCentric
          && error_type_ == ErrorType::LeftInvariant)) {
    G.block(0, 0, dimP - dimTheta, dimP - dimTheta)
        = Adjoint_SEK3(state.get_world_X());
  }

  // Continuous noise covariance
  Eigen::MatrixXd Qc = Eigen::MatrixXd::Zero(
      dimP, dimP);    // Landmark noise terms will remain zero
  Qc.block<3, 3>(0, 0) = gyro_cov_;
  Qc.block<3, 3>(3, 3) = accel_cov_;

  /// TODO: make sure contact covariance is in the euclidean space
  // for (auto& column_id_to_aug_type : *(state.get_matrix_idx_map().get())) {
  //   Qc.block<3, 3>(3 + 3 * (column_id_to_aug_type.first - 3),
  //                  3 + 3 * (column_id_to_aug_type.first - 3))
  //       = noise_params_.get_augment_cov(
  //           column_id_to_aug_type.second);    // Augment state noise terms
  // }


  Qc.block<3, 3>(dimP - dimTheta, dimP - dimTheta) = gyro_bias_cov_;
  Qc.block<3, 3>(dimP - dimTheta + 3, dimP - dimTheta + 3) = accel_bias_cov_;

  // Noise Covariance Discretization
  Eigen::MatrixXd PhiG = Phi * G;
  Eigen::MatrixXd Qd = PhiG * Qc * PhiG.transpose()
                       * dt;    // Approximated discretized noise
                                // matrix (TODO: compute analytical)
  return Qd;
}

void ImuPropagation::InitImuBias() {
  if (!static_bias_initialization_) {
    bias_initialized_ = true;
    std::cout << "Bias inialization is set to false." << std::endl;
    std::cout << "Bias is initialized using prior as: \n" << bg0_ << std::endl;
    std::cout << ba0_ << std::endl;
    return;
  }


  // Initialize bias based on imu orientation and static assumption
  if (bias_init_vec_.size() < init_bias_size_) {
    sensor_data_buffer_mutex_ptr_.get()->lock();
    if (sensor_data_buffer_ptr_->empty()) {
      sensor_data_buffer_mutex_ptr_.get()->unlock();
      return;
    }
    auto imu_measurement = *(sensor_data_buffer_ptr_->front().get());
    sensor_data_buffer_ptr_->pop();
    sensor_data_buffer_mutex_ptr_.get()->unlock();

    Eigen::Vector3d w = imu_measurement.get_ang_vel();    // Angular Velocity
    Eigen::Vector3d a = imu_measurement.get_lin_acc();    // Linear Acceleration

    // Rotate imu frame to align it with the body frame:
    w = R_imu2body_ * w;
    a = R_imu2body_ * a;

    Eigen::Quaternion<double> quat = imu_measurement.get_quaternion();
    Eigen::Matrix3d R;
    if (use_imu_ori_est_init_bias_) {
      R = quat.toRotationMatrix();
    } else {
      R = Eigen::Matrix3d::Identity();
    }

    a = (R.transpose() * (R * a + g_)).eval();
    Eigen::Matrix<double, 6, 1> v;
    v << w(0), w(1), w(2), a(0), a(1), a(2);
    bias_init_vec_.push_back(v);    // Store imu data with gravity removed
  } else {
    // Compute average bias of stored data
    Eigen::Matrix<double, 6, 1> avg = Eigen::Matrix<double, 6, 1>::Zero();
    for (int i = 0; i < bias_init_vec_.size(); ++i) {
      avg = (avg + bias_init_vec_[i]).eval();
    }
    avg = (avg / bias_init_vec_.size()).eval();
    std::cout << "IMU bias initialized to: " << avg.transpose() << std::endl;
    bg0_ = avg.head<3>();
    ba0_ = avg.tail<3>();
    bias_initialized_ = true;
  }
}

Eigen::Matrix3d ImuPropagation::compute_R_imu2body(
    const std::vector<double> imu2body) {
  Eigen::Quaternion<double> quarternion_imu2body(imu2body[0], imu2body[1],
                                                 imu2body[2], imu2body[3]);
  Eigen::Matrix3d R_imu2body;
  R_imu2body = quarternion_imu2body.toRotationMatrix();
  return R_imu2body;
}

const Eigen::Vector3d ImuPropagation::get_estimate_gyro_bias() const {
  return bg0_;
}

const Eigen::Vector3d ImuPropagation::get_estimate_accel_bias() const {
  return ba0_;
}

const bool ImuPropagation::get_bias_initialized() const {
  return bias_initialized_;
}

}    // namespace inekf