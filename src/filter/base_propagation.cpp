#include "filter/base_propagation.h"

using namespace std;

// Base propagation class
// ======================================================================
// Default constructor
Propagation::Propagation(const NoiseParams& params)
    : g_((Eigen::VectorXd(3) << 0, 0, -9.81).finished()),
      magnetic_field_(
          (Eigen::VectorXd(3) << std::cos(1.2049), 0, std::sin(1.2049))
              .finished()),
      noise_params_(params) {
  propagation_type_ = PropagationType::BASE;
}

// Base method for propagation
void Propagation::Propagate(RobotState& state) {
  // Just a skeleton, to be implemented in the child class
}

// Return measurement queue
MeasurementQueuePtr Propagation::get_sensor_data_buffer_ptr() {
  return nullptr;
}

// Return propagation type
const PropagationType Propagation::get_propagation_type() const {
  return propagation_type_;
}

// Return noise params
const NoiseParams Propagation::get_noise_params() const {
  return noise_params_;
}
