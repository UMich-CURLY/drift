#include "inekf/inekf_propagate.h"

namespace inekf {
using namespace std;
using namespace lie_group;

// Default constructor
Propagation::Propagation(std::shared_ptr<sensor_data_t> sensor_data_buffer, NoiseParams params, ErrorType error_type)
    : g_((Eigen::VectorXd(3) << 0, 0, -9.81).finished()),
      magnetic_field_((Eigen::VectorXd(3) << std::cos(1.2049), 0, std::sin(1.2049)).finished()),
      noise_params_(params),
      error_type_(error_type) {
    sensor_data_buffer_ = sensor_data_buffer;
}

// InEKF Propagation - Inertial Data
void Propagation::Propagate(const Eigen::Matrix<double, 6, 1>& imu, double dt, RobotState& state) {
    // Bias corrected IMU measurements
    Eigen::Vector3d w = imu.head(3) - state.getGyroscopeBias();        // Angular Velocity
    Eigen::Vector3d a = imu.tail(3) - state.getAccelerometerBias();    // Linear Acceleration

    // Get current state estimate and dimensions
    Eigen::MatrixXd X = state.getX();
    Eigen::MatrixXd Xinv = state.Xinv();
    Eigen::MatrixXd P = state.getP();
    int dimX = state.dimX();
    int dimP = state.dimP();
    int dimTheta = state.dimTheta();

    //  ------------ Propagate Covariance --------------- //
    Eigen::MatrixXd Phi = this->StateTransitionMatrix(w, a, dt, state);
    Eigen::MatrixXd Qd = this->DiscreteNoiseMatrix(Phi, dt, state);
    Eigen::MatrixXd P_pred = Phi * P * Phi.transpose() + Qd;

    // If we don't want to estimate bias, remove correlation
    if (!estimate_bias_) {
        P_pred.block(0, dimP - dimTheta, dimP - dimTheta, dimTheta) = Eigen::MatrixXd::Zero(dimP - dimTheta, dimTheta);
        P_pred.block(dimP - dimTheta, 0, dimTheta, dimP - dimTheta) = Eigen::MatrixXd::Zero(dimTheta, dimP - dimTheta);
        P_pred.block(dimP - dimTheta, dimP - dimTheta, dimTheta, dimTheta)
            = Eigen::MatrixXd::Identity(dimTheta, dimTheta);
    }

    //  ------------ Propagate Mean --------------- //
    Eigen::Matrix3d R = state.getRotation();
    Eigen::Vector3d v = state.getVelocity();
    Eigen::Vector3d p = state.getPosition();

    Eigen::Vector3d phi = w * dt;
    Eigen::Matrix3d G0 = Gamma_SO3(phi,
                                   0);    // Computation can be sped up by computing G0,G1,G2 all at once
    Eigen::Matrix3d G1 = Gamma_SO3(phi, 1);
    Eigen::Matrix3d G2 = Gamma_SO3(phi, 2);

    Eigen::MatrixXd X_pred = X;
    if (state.getStateType() == StateType::WorldCentric) {
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
        X_pred.block<3, 1>(0, 4) = G0t * (p + v * dt - (G2 * a + 0.5 * R * g_) * dt * dt);
        for (int i = 5; i < dimX; ++i) {
            X_pred.block<3, 1>(0, i) = G0t * X.block<3, 1>(0, i);
        }
    }

    //  ------------ Update State --------------- //
    state.setX(X_pred);
    state.setP(P_pred);
}

// Return noise params
NoiseParams Propagation::get_noise_params() const { return noise_params_; }

// Sets the filter's noise parameters
void Propagation::set_noise_params(NoiseParams params) { noise_params_ = params; }

// Compute Analytical state transition matrix
Eigen::MatrixXd Propagation::StateTransitionMatrix(Eigen::Vector3d& w, Eigen::Vector3d& a, double dt,
                                                   const RobotState& state) {
    Eigen::Vector3d phi = w * dt;
    Eigen::Matrix3d G0 = Gamma_SO3(phi,
                                   0);         // Computation can be sped up by computing G0,G1,G2 all at once
    Eigen::Matrix3d G1 = Gamma_SO3(phi, 1);    // TODO: These are also needed for the mean
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
          * (ax * G2t * dt2 + ((sinthetadt - thetadtcosthetadt) / (theta3)) * (wx * ax)
             - ((cos2thetadt - 4 * costhetadt + 3) / (4 * theta4)) * (wx * ax * wx)
             + ((4 * sinthetadt + sin2thetadt - 4 * thetadtcosthetadt - 2 * thetadt) / (4 * theta5)) * (wx * ax * wx2)
             + ((thetadt2 - 2 * thetadtsinthetadt - 2 * costhetadt + 2) / (2 * theta4)) * (wx2 * ax)
             - ((6 * thetadt - 8 * sinthetadt + sin2thetadt) / (4 * theta5)) * (wx2 * ax * wx)
             + ((2 * thetadt2 - 4 * thetadtsinthetadt - cos2thetadt + 1) / (4 * theta6)) * (wx2 * ax * wx2));

    Eigen::Matrix3d Phi35L
        = G0t
          * (ax * G3t * dt3 - ((thetadtsinthetadt + 2 * costhetadt - 2) / (theta4)) * (wx * ax)
             - ((6 * thetadt - 8 * sinthetadt + sin2thetadt) / (8 * theta5)) * (wx * ax * wx)
             - ((2 * thetadt2 + 8 * thetadtsinthetadt + 16 * costhetadt + cos2thetadt - 17) / (8 * theta6))
                   * (wx * ax * wx2)
             + ((thetadt3 + 6 * thetadt - 12 * sinthetadt + 6 * thetadtcosthetadt) / (6 * theta5)) * (wx2 * ax)
             - ((6 * thetadt2 + 16 * costhetadt - cos2thetadt - 15) / (8 * theta6)) * (wx2 * ax * wx)
             + ((4 * thetadt3 + 6 * thetadt - 24 * sinthetadt - 3 * sin2thetadt + 24 * thetadtcosthetadt)
                / (24 * theta7))
                   * (wx2 * ax * wx2));


    // TODO: Get better approximation using taylor series when theta < tol
    const double tol = 1e-6;
    if (theta < tol) {
        Phi25L = (1 / 2) * ax * dt2;
        Phi35L = (1 / 6) * ax * dt3;
    }

    // Fill out analytical state transition matrices
    if ((state.getStateType() == StateType::WorldCentric && error_type_ == ErrorType::LeftInvariant)
        || (state.getStateType() == StateType::BodyCentric && error_type_ == ErrorType::RightInvariant)) {
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
        Eigen::Matrix3d R = state.getRotation();
        Eigen::Vector3d v = state.getVelocity();
        Eigen::Vector3d p = state.getPosition();
        Eigen::Matrix3d RG0 = R * G0;
        Eigen::Matrix3d RG1dt = R * G1 * dt;
        Eigen::Matrix3d RG2dt2 = R * G2 * dt2;
        Phi.block<3, 3>(3, 0) = gx * dt;                                                                // Phi_21
        Phi.block<3, 3>(6, 0) = 0.5 * gx * dt2;                                                         // Phi_31
        Phi.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity() * dt;                                       // Phi_32
        Phi.block<3, 3>(0, dimP - dimTheta) = -RG1dt;                                                   // Phi_15
        Phi.block<3, 3>(3, dimP - dimTheta) = -skew(v + RG1dt * a + g_ * dt) * RG1dt + RG0 * Phi25L;    // Phi_25
        Phi.block<3, 3>(6, dimP - dimTheta)
            = -skew(p + v * dt + RG2dt2 * a + 0.5 * g_ * dt2) * RG1dt + RG0 * Phi35L;    // Phi_35
        for (int i = 5; i < dimX; ++i) {
            Phi.block<3, 3>((i - 2) * 3, dimP - dimTheta) = -skew(state.getVector(i)) * RG1dt;    // Phi_(3+i)5
        }
        Phi.block<3, 3>(3, dimP - dimTheta + 3) = -RG1dt;     // Phi_26
        Phi.block<3, 3>(6, dimP - dimTheta + 3) = -RG2dt2;    // Phi_36
    }
    return Phi;
}

// Compute Discrete noise matrix
template<int dim>
Eigen::MatrixXd Propagation::DiscreteNoiseMatrix(Eigen::MatrixXd& Phi, double dt, const RobotState& state) {
    int dimX = state.dimX();
    int dimTheta = state.dimTheta();
    int dimP = state.dimP();
    Eigen::MatrixXd G = Eigen::MatrixXd::Identity(dimP, dimP);

    // Compute G using Adjoint of Xk if needed, otherwise identity (Assumes
    // unpropagated state)
    if ((state.getStateType() == StateType::WorldCentric && error_type_ == ErrorType::RightInvariant)
        || (state.getStateType() == StateType::BodyCentric && error_type_ == ErrorType::LeftInvariant)) {
        G.block(0, 0, dimP - dimTheta, dimP - dimTheta) = Adjoint_SEK3(state.getWorldX());
    }

    // Continuous noise covariance
    Eigen::MatrixXd Qc = Eigen::MatrixXd::Zero(dimP, dimP);    // Landmark noise terms will remain zero
    Qc.block<dim, dim>(0, 0) = noise_params_.getGyroscopeCov();
    Qc.block<dim, dim>(dim, dim) = noise_params_.getAccelerometerCov();

    /// TODO: build get_augmented_map() function
    // std::map<std::string, int> augmented_states = state.get_augmented_map();
    std::map<std::string, int> augmented_states = {};

    // add a map of augment states for loop
    for (std::map<std::string, int>::const_iterator it = augmented_states.begin(); it != augmented_states.end(); ++it) {
        Qc.block<dim, dim>(dim + dim * (it->second - dim), dim + dim * (it->second - dim))
            = noise_params_.getAugmentCov();    // Augment state noise terms
    }                                           // TODO: Use kinematic orientation to map noise from augment state
                                                // frame to body frame (not needed if noise is isotropic)

    Qc.block<3, 3>(dimP - dimTheta, dimP - dimTheta) = noise_params_.getGyroscopeBiasCov();
    Qc.block<3, 3>(dimP - dimTheta + dim, dimP - dimTheta + dim) = noise_params_.getAccelerometerBiasCov();

    // Noise Covariance Discretization
    Eigen::MatrixXd PhiG = Phi * G;
    Eigen::MatrixXd Qd = PhiG * Qc * PhiG.transpose() * dt;    // Approximated discretized noise matrix
                                                               // (TODO: compute analytical)
    return Qd;
}


}    // namespace inekf
