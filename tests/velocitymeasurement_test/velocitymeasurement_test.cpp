#include <gtest/gtest.h>
#include "drift/measurement/velocity.h"

using namespace measurement;

TEST(VelocityMeasurementTest, Ctor) {
  VelocityMeasurement<double> velocity_data;
  EXPECT_EQ(velocity_data.get_type(), 3);
  EXPECT_EQ(velocity_data.get_type(), VELOCITY);
  EXPECT_EQ(velocity_data.get_velocity().x(), 0);
  EXPECT_EQ(velocity_data.get_velocity().y(), 0);
  EXPECT_EQ(velocity_data.get_velocity().z(), 0);
}


TEST(VelocityMeasurementTest, VelSetGetBasic) {
  VelocityMeasurement<double> velocity_data;

  velocity_data.set_velocity(1.234, 5.678, 9.012);

  EXPECT_EQ(velocity_data.get_velocity().x(), 1.234);
  EXPECT_EQ(velocity_data.get_velocity().y(), 5.678);
  EXPECT_EQ(velocity_data.get_velocity().z(), 9.012);
}

TEST(VelocityMeasurementTest, VelMag) {
  VelocityMeasurement<double> velocity_data;
  velocity_data.set_velocity(3, 4, 0);
  EXPECT_EQ(velocity_data.get_vel_mag(), 5);
}

TEST(VelocityMeasurementTest, VelVec) {
  VelocityMeasurement<double> velocity_data;

  double tol = 1e-5;

  velocity_data.set_velocity(3, 4, 0);
  double mag = velocity_data.get_vel_unit_vec().norm();
  EXPECT_EQ(velocity_data.get_vel_unit_vec()[0], 0.6);
  EXPECT_EQ(velocity_data.get_vel_unit_vec()[1], 0.8);
  EXPECT_EQ(velocity_data.get_vel_unit_vec()[2], 0);
  EXPECT_NEAR(mag, 1, tol);

  velocity_data.set_velocity(5, 6, 7);
  EXPECT_NEAR(velocity_data.get_vel_mag(), 10.488088, tol);
  EXPECT_NEAR(velocity_data.get_vel_unit_vec()[0], 0.476731, tol);
  EXPECT_NEAR(velocity_data.get_vel_unit_vec()[1], 0.572078, tol);
  EXPECT_NEAR(velocity_data.get_vel_unit_vec()[2], 0.667424, tol);

  velocity_data.set_velocity(23423, -345345, -456456);
  EXPECT_NEAR(velocity_data.get_vel_mag(), 572855.903251, tol);
  EXPECT_NEAR(velocity_data.get_vel_unit_vec()[0], 0.0408881, tol);
  EXPECT_NEAR(velocity_data.get_vel_unit_vec()[1], -0.6028479, tol);
  EXPECT_NEAR(velocity_data.get_vel_unit_vec()[2], -0.7968077, tol);
  mag = velocity_data.get_vel_unit_vec().norm();
  EXPECT_NEAR(mag, 1, tol);
}
