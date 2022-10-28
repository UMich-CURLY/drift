#include "inekf/inekf_propagate.h"

namespace inekf {
using namespace std;
using namespace lie_group;

Propagation::Propagation() {
    // Initialize the filter
}

// InEKF Propagation - Inertial Data
void Propagation::Propagate(const Eigen::Matrix<double,6,1>& imu, double dt, RobotState& state) {

    // Bias corrected IMU measurements
    Eigen::Vector3d w = imu.head(3)  - state.getGyroscopeBias();    // Angular Velocity
    Eigen::Vector3d a = imu.tail(3) - state.getAccelerometerBias(); // Linear Acceleration
    
    // Get current state estimate and dimensions
    Eigen::MatrixXd X = state.getX();
    Eigen::MatrixXd Xinv = state.Xinv();
    Eigen::MatrixXd P = state.getP();
    int dimX = state.dimX();
    int dimP = state.dimP();
    int dimTheta = state.dimTheta();

    //  ------------ Propagate Covariance --------------- //
    this->set_state(state);
    Eigen::MatrixXd Phi = this->StateTransitionMatrix(w,a,dt);
    Eigen::MatrixXd Qd = this->DiscreteNoiseMatrix(Phi, dt);
    Eigen::MatrixXd P_pred = Phi * P * Phi.transpose() + Qd;

    // If we don't want to estimate bias, remove correlation
    if (!estimate_bias_) {
        P_pred.block(0,dimP-dimTheta,dimP-dimTheta,dimTheta) = Eigen::MatrixXd::Zero(dimP-dimTheta,dimTheta);
        P_pred.block(dimP-dimTheta,0,dimTheta,dimP-dimTheta) = Eigen::MatrixXd::Zero(dimTheta,dimP-dimTheta);
        P_pred.block(dimP-dimTheta,dimP-dimTheta,dimTheta,dimTheta) = Eigen::MatrixXd::Identity(dimTheta,dimTheta);
    }    

    //  ------------ Propagate Mean --------------- // 
    Eigen::Matrix3d R = state.getRotation();
    Eigen::Vector3d v = state.getVelocity();
    Eigen::Vector3d p = state.getPosition();

    Eigen::Vector3d phi = w*dt;
    Eigen::Matrix3d G0 = Gamma_SO3(phi,0); // Computation can be sped up by computing G0,G1,G2 all at once
    Eigen::Matrix3d G1 = Gamma_SO3(phi,1);
    Eigen::Matrix3d G2 = Gamma_SO3(phi,2);

    Eigen::MatrixXd X_pred = X;
    if (state.getStateType() == StateType::WorldCentric) {
        // Propagate world-centric state estimate
        X_pred.block<3,3>(0,0) = R * G0;
        X_pred.block<3,1>(0,3) = v + (R*G1*a + g_)*dt;
        X_pred.block<3,1>(0,4) = p + v*dt + (R*G2*a + 0.5*g_)*dt*dt;
    } else {
        // Propagate body-centric state estimate
        Eigen::MatrixXd X_pred = X;
        Eigen::Matrix3d G0t = G0.transpose();
        X_pred.block<3,3>(0,0) = G0t*R;
        X_pred.block<3,1>(0,3) = G0t*(v - (G1*a + R*g_)*dt);
        X_pred.block<3,1>(0,4) = G0t*(p + v*dt - (G2*a + 0.5*R*g_)*dt*dt);
        for (int i=5; i<dimX; ++i) {
            X_pred.block<3,1>(0,i) = G0t*X.block<3,1>(0,i);
        }
    } 

    //  ------------ Update State --------------- // 
    state.setX(X_pred);
    state.setP(P_pred);      

}


} // end inekf namespace
