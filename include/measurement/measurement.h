#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <iostream>
#include <string>

#include <eigen3/Eigen/Dense>

enum MeasurementType { EMPTY, IMU, KINEMATICS, CONTACT, JOINT_STATE };

class Measurement {
  struct MeasurementHeader {
    uint64_t seq;
    double stamp;
    std::string frame_id;
  };

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Measurement();
  Measurement(MeasurementType type);
  virtual ~Measurement() = default;

  MeasurementHeader header;

  /**
   * @brief Get the timestamp value for the measurement.
   *
   * @return double: The timestamp.
   */
  double get_time();

  /**
   * @brief Get the measurement type.
   *
   * @return MeasurementType: The measurement type.
   */
  MeasurementType get_type();

  friend std::ostream& operator<<(std::ostream& os, const Measurement& m);

 protected:
  MeasurementType type_;
};

struct MeasurementCompare {
  bool operator()(Measurement& lhs, Measurement& rhs) const {
    return lhs.get_time() > rhs.get_time();
  }
};

#endif
