#pragma once
#include <stdint.h>
#include <string>
#include "measurement.h"

// namespace cheetah_inekf_lcm {

template<typename T>
class KinematicsMeasurement : public Measurement {
 public:
  // Construct KINEMATICS measurement
  KinematicsMeasurement() { type_ = KINEMATICS; }

  void set_kinematics_array(const inekf_msgs::KinematicsArray& kinematics) {
    kin_arr = kinematics;
  }

  // const inekf_msgs::KinematicsArray& getKinematicsArray() {
  //     return kin_arr;
  // }


 private:
  std::vector<T> position_;
  std::vector<T> velocity_;
  std::vector<T> effort_;
  // inekf_msgs::KinematicsArray kin_arr;
};
// template class cheetah_inekf_lcm::lcm_handler<12>;
// }    // namespace cheetah_inekf_lcm