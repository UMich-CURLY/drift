// #include "filter/inekf/propagation/base_propagation.h"
#include "math/lie_group.h"

namespace inekf {
using namespace std;
using namespace lie_group;

// Base propagation class
// ======================================================================
// Default constructor
Propagation::Propagation(NoiseParams params, ErrorType error_type)
    : g_((Eigen::VectorXd(3) << 0, 0, -9.81).finished()),
      magnetic_field_(
          (Eigen::VectorXd(3) << std::cos(1.2049), 0, std::sin(1.2049))
              .finished()),
      noise_params_(params),
      error_type_(error_type) {}

// Base method for propagation
void Propagation::Propagate(RobotState& state, double dt) {
  // Just a skeleton, to be implemented in the child class
}

// Return noise params
NoiseParams Propagation::get_noise_params() const { return noise_params_; }

// Sets the filter's noise parameters
void Propagation::set_noise_params(NoiseParams params) {
  noise_params_ = params;
}


}    // namespace inekf
