#include <gtest/gtest.h>
#include "drift/measurement/contact.h"

using namespace measurement;

TEST(ContactMeasurementTest, Ctor) {
  ContactMeasurement contact_data;
  EXPECT_EQ(contact_data.get_type(), 5);
  EXPECT_EQ(contact_data.get_type(), CONTACT);
}

TEST(ContactMeasurementTest, ContactSetGetBasic) {
  ContactMeasurement contact_data;
  Eigen::Matrix<bool, 6, 1> ct;
  ct << 1, 0, 1, 0, 1, 1;
  contact_data.set_contact(ct);
  EXPECT_EQ(contact_data.get_contact().size(), 6);
  EXPECT_EQ(contact_data.get_contact()[0], 1);
  EXPECT_EQ(contact_data.get_contact()[1], 0);
  EXPECT_EQ(contact_data.get_contact()[2], 1);
  EXPECT_EQ(contact_data.get_contact()[3], 0);
  EXPECT_EQ(contact_data.get_contact()[4], 1);
  EXPECT_EQ(contact_data.get_contact()[5], 1);
}
