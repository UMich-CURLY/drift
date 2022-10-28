#include <gtest/gtest.h>
#include "measurement/contact.h"

TEST(ContactMeasurementTest, Ctor) {
  ContactMeasurement<4> contact_data;
  EXPECT_EQ(contact_data.get_type(), 5);
  EXPECT_EQ(contact_data.get_type(), CONTACT);

  EXPECT_EQ(contact_data.get_contacts().size(), 4);
  EXPECT_EQ(contact_data.get_contacts()[0], 0);
  EXPECT_EQ(contact_data.get_contacts()[1], 0);
  EXPECT_EQ(contact_data.get_contacts()[2], 0);
  EXPECT_EQ(contact_data.get_contacts()[3], 0);
}

TEST(ContactMeasurementTest, ContactSetGetBasic) {
  ContactMeasurement<6> contact_data;
  Eigen::Matrix<bool, 6, 1> ct;
  ct << 1, 0, 1, 0, 1, 1;
  contact_data.set_contacts(ct);
  EXPECT_EQ(contact_data.get_contacts().size(), 6);
  EXPECT_EQ(contact_data.get_contacts()[0], 1);
  EXPECT_EQ(contact_data.get_contacts()[1], 0);
  EXPECT_EQ(contact_data.get_contacts()[2], 1);
  EXPECT_EQ(contact_data.get_contacts()[3], 0);
  EXPECT_EQ(contact_data.get_contacts()[4], 1);
  EXPECT_EQ(contact_data.get_contacts()[5], 1);
}
