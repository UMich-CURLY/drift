/**
 *  @file   contact.h
 *  @author Justin Yu
 *  @brief  Header file for robot ground contact state measurement
 *  @date   Nov 16, 2022
 **/

#ifndef CONTACT_H
#define CONTACT_H

#include "measurement.h"

template<unsigned int CONTACT_DIM>
class ContactMeasurement : public Measurement {
 public:
  ContactMeasurement();    // default constructor

  /**
   * @brief Set the contact state vector for this measurement.
   *
   * @param[in] Eigen::Matrix: vector of booleans containing contact state with
   * length CONTACT_DIM.
   */
  void set_contact(const Eigen::Matrix<bool, CONTACT_DIM, 1>& contacts);

  /**
   * @brief Get the contact state vector for this measurement.
   *
   * @return Eigen::Matrix: vector of booleans containing contact state with
   * length CONTACT_DIM.
   */
  Eigen::Matrix<bool, CONTACT_DIM, 1> get_contact() const;

 private:
  Eigen::Matrix<bool, CONTACT_DIM, 1> contacts_;
};
#include "measurement/impl/contact_impl.cpp"

#endif    // CONTACT_H
