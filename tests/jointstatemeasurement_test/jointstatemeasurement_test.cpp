#include <gtest/gtest.h>
#include "measurement/joint_state.h"

TEST(JointStateMeasurementTest, Ctor) {
  JointStateMeasurement<12> joint_data;
  EXPECT_EQ(joint_data.get_type(), 4);
  EXPECT_EQ(joint_data.get_type(), JOINT_STATE);

  EXPECT_EQ(joint_data.get_joint_pos().size(), 12);
  for (size_t i = 0; i < 12; i++) {
    EXPECT_EQ(joint_data.get_joint_pos()[i], 0);
    EXPECT_EQ(joint_data.get_joint_vel()[i], 0);
    EXPECT_EQ(joint_data.get_joint_effort()[i], 0);
  }
}
