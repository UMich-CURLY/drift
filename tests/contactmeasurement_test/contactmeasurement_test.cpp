#include <gtest/gtest.h>
#include "measurement/contact.h"

TEST(ContactMeasurementTest, Ctor) {
  ContactMeasurement<4> contact_data;
  EXPECT_EQ(contact_data.get_type(), 5);
  EXPECT_EQ(contact_data.get_type(), CONTACT);

  EXPECT_EQ(contact_data.get_contact().size(), 4);
  for (size_t i = 0; i < 4; i++) {
    EXPECT_EQ(contact_data.get_contact()[i], 0);
  }
}

TEST(ContactMeasurementTest, ContactSetGetBasic) {
  ContactMeasurement<6> contact_data;
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
