#include "measurement/measurement.h"

using namespace std;

// using namespace inekf;

Measurement::Measurement() : type_(EMPTY) { header.stamp = 0; }

Measurement::Measurement(MeasurementType type) : type_(type) {
  header.stamp = 0;
}

double Measurement::get_time() { return header.stamp; }

MeasurementType Measurement::get_type() { return type_; }

ostream& operator<<(ostream& os, const Measurement& m) {
  string type_str;
  switch (m.type_) {
    case IMU:
      type_str = "IMU";
      break;
    default:
      type_str = "Unknown";
  }
  os << "Measurement type: " << type_str << endl;
  return os;
}
