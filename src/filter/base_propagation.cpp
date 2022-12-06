#include "filter/base_propagation.h"

using namespace std;

// Base propagation class
// ======================================================================
// Default constructor
Propagation::Propagation(const NoiseParams& params,
                         const bool estimate_bias = true)
    : g_((Eigen::VectorXd(3) << 0, 0, -9.81).finished()),
      magnetic_field_(
          (Eigen::VectorXd(3) << std::cos(1.2049), 0, std::sin(1.2049))
              .finished()),
      noise_params_(params),
      estimate_bias_(estimate_bias) {}

// Base method for propagation
void Propagation::Propagate(RobotState& state, double dt) {
  // Just a skeleton, to be implemented in the child class
}

// Return noise params
const NoiseParams Propagation::get_noise_params() const {
  return noise_params_;
}
