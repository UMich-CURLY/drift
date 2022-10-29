#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "measurement.h"

template<typename T>
class KinematicsMeasurement : public Measurement {
 public:
  KinematicsMeasurement();

  void set_kinematics_array(const& kinematics);

 private:
  std::vector<T> position_;
  std::vector<T> velocity_;
  std::vector<T> effort_;
};

#endif