#include <gtest/gtest.h>
#include "drift/measurement/joint_state.h"

#define tol2 1e-9

using namespace measurement;

TEST(JointStateMeasurementTest, Ctor) {
  JointStateMeasurement<double> joint_data;
  EXPECT_EQ(joint_data.get_type(), 4);
  EXPECT_EQ(joint_data.get_type(), JOINT_STATE);
}

TEST(JointStateMeasurementTest, SetGetBasic) {
  JointStateMeasurement<double> joint_data;
  Eigen::Vector3d v;
  Eigen::Vector3d p;
  Eigen::Vector3d e;
  v << 0.123, 0.234, 0.345;
  p << 0.456, 0.567, 0.678;
  e << 0.789, 0.900, 1.011;

  joint_data.set_joint_state(v, p, e);

  EXPECT_EQ(joint_data.get_joint_pos().size(), 3);

  for (size_t i = 0; i < 3; i++) {
    EXPECT_NEAR(joint_data.get_joint_pos()[i], 0.123 + i * 0.111, tol2);
    EXPECT_NEAR(joint_data.get_joint_vel()[i], 0.456 + i * 0.111, tol2);
    EXPECT_NEAR(joint_data.get_joint_effort()[i], 0.789 + i * 0.111, tol2);
  }

  joint_data.set_encoders(e);
  for (size_t i = 0; i < 3; i++) {
    EXPECT_NEAR(joint_data.get_joint_pos()[i], 0.789 + i * 0.111, tol2);
  }
}
