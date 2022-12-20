#include "filter/base_correction.h"

using namespace std;

// Constructor with error type

Correction::Correction()
    : g_((Eigen::VectorXd(3) << 0, 0, -9.81).finished()),
      magnetic_field_((Eigen::VectorXd(3) << 0, 0, 0).finished()) {
  correction_type_ = CorrectionType::BASE;
}

void Correction::Correct(RobotState& state) {
  // Just a skeleton, to be implemented in the child class
}

CorrectionType Correction::get_correction_type() { return correction_type_; }