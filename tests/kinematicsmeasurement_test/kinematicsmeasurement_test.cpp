#include <gtest/gtest.h>
#include "measurement/kinematics.h"

#define tol 1e-9

TEST(KinematicsMeasurementTest, Ctor) {
  KinematicsMeasurement<double> kin_data;
  EXPECT_EQ(kin_data.get_type(), 2);
  EXPECT_EQ(kin_data.get_type(), KINEMATICS);

  for (size_t i = 0; i < 3; i++) {
    EXPECT_EQ(kin_data.get_kin_pos()[i], 0);
    EXPECT_EQ(kin_data.get_kin_vel()[i], 0);
    EXPECT_EQ(kin_data.get_kin_effort()[i], 0);
  }
}

TEST(KinematicsMeasurementTest, SetGetBasic) {
  KinematicsMeasurement<double> kin_data;
  Eigen::Vector3d v;
  Eigen::Vector3d p;
  Eigen::Vector3d e;
  v << 0.123, 0.234, 0.345;
  p << 0.456, 0.567, 0.678;
  e << 0.789, 0.900, 1.011;

  kin_data.set_kin_state(v, p, e);

  EXPECT_EQ(kin_data.get_kin_pos().size(), 3);

  for (size_t i = 0; i < 3; i++) {
    EXPECT_NEAR(kin_data.get_kin_pos()[i], 0.123 + i * 0.111, tol);
    EXPECT_NEAR(kin_data.get_kin_vel()[i], 0.456 + i * 0.111, tol);
    EXPECT_NEAR(kin_data.get_kin_effort()[i], 0.789 + i * 0.111, tol);
  }
}
