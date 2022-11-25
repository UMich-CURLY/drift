// #include "filter/inekf/correction/base_correction.h"
// #include "math/lie_group.h"

namespace inekf {
using namespace std;
using namespace lie_group;

// Constructor with error type

Correction::Correction(ErrorType error_type)
    : g_((Eigen::VectorXd(3) << 0, 0, -9.81).finished()),
      magnetic_field_((Eigen::VectorXd(3) << 0, 0, 0).finished()),
      error_type_(error_type) {}

void Correction::Correct(RobotState& state) {
  // Just a skeleton, to be implemented in the child class
}

}    // namespace inekf