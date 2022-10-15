#ifndef CONTACTS_H
#define CONTACTS_H

#include <stdint.h>
#include <string>
#include "measurement/measurement.h"

class ContactsMeasurement : public Measurement {
 public:
  ContactsMeasurement();

  void setContacts(const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts);

  Eigen::Matrix<bool, Eigen::Dynamic, 1> getContacts() { return contacts_; }

 private:
  Eigen::Matrix<bool, Eigen::Dynamic, 1> contacts_;
};
#include "measurement/impl/contacts_impl.cpp"

#endif