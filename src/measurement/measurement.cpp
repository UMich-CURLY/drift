#include "measurement/measurement.h"

using namespace std;

Measurement::Measurement() : type_(EMPTY) { header.stamp = 0; }

Measurement::Measurement(MeasurementType type) : type_(type) {
  header.stamp = 0;
}
void Measurement::set_time(double t) { header.stamp = t; }

void Measurement::set_header(const MeasurementHeader& h) { header = h; }

double Measurement::get_time() const { return header.stamp; }

MeasurementType Measurement::get_type() const { return type_; }

ostream& operator<<(ostream& os, const Measurement& m) {
  string type_str;
  switch (m.type_) {
    case IMU:
      type_str = "IMU";
      break;
    case KINEMATICS:
      type_str = "KINEMATICS";
      break;
    case VELOCITY:
      type_str = "VELOCITY";
      break;
    case JOINT_STATE:
      type_str = "JOINT_STATE";
      break;
    case CONTACT:
      type_str = "CONTACT";
      break;
    default:
      type_str = "UNKNOWN";
  }
  os << "Measurement type: " << type_str << endl;
  return os;
}
