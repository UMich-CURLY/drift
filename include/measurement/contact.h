#ifndef IMU_H
#define IMU_H

#include "measurement.h"

template<int NUMCONTACTS>
class ContactMeasurement : public Measurement {
 public:
  ContactMeasurement();

  /**
   * @brief Set the contact state vector for this measurement.
   *
   * @param[in] Eigen::Matrix: n-vector of booleans containing contact state.
   */
  void set_contact(const Eigen::Matrix<bool, NUMCONTACTS, 1>& contacts);

  /**
   * @brief Get the contact state vector for this measurement.
   *
   * @return Eigen::Matrix: n-vector of booleans containing contact state.
   */
  Eigen::Matrix<bool, NUMCONTACTS, 1> get_contact();

 private:
  Eigen::Matrix<bool, NUMCONTACTS, 1> contacts_;
};
#include "measurement/impl/contact_impl.cpp"

#endif
