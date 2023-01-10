#include <gtest/gtest.h>
#include <iostream>
#include "kinematics/mini_cheetah_kinematics.h"

#define tol3 1e-9

TEST(cheetahkinematicstest, Ctor) {
  MiniCheetahKin kin_data;
  EXPECT_EQ(kin_data.get_type(), 2);
  EXPECT_EQ(kin_data.get_type(), KINEMATICS);

  for (size_t i = 0; i < 3; i++) {
    EXPECT_EQ(kin_data.get_kin_pos(1)(i), 0);
    EXPECT_EQ(kin_data.get_kin_vel(1)(i), 0);
  }
  /*
    EXPECT_EQ(kin_data.get_J(1).rows(), 3);
    EXPECT_EQ(kin_data.get_J(1).cols(), 3);
    for (size_t i = 0; i < 9; i++) {
      EXPECT_EQ(kin_data.get_J(1)(i), 0);
    }
  */
  for (size_t i = 0; i < 4; i++) {
    EXPECT_EQ(kin_data.get_contact(i), 0);
  }

  for (size_t i = 0; i < 12; i++) {
    EXPECT_EQ(kin_data.get_joint_state()(i), 0);
  }
}

TEST(cheetahkinematicstest, JointStateSet) {
  MiniCheetahKin kin_data;
  Eigen::Matrix<double, 12, 1> v;
  v << 0.123, 0.234, 0.345, 0.456, 0.567, 0.678, 0.789, 0.900, 1.011, 1.123,
      1.234, 1.345;
  kin_data.set_joint_state(v);
  std::cout << kin_data.get_joint_state();
}

TEST(cheetahkinematicstest, OverloadCtor) {
  Eigen::Matrix<double, 12, 1> js;
  Eigen::Matrix<bool, 4, 1> ct;
  js << 0.123, 0.234, 0.345, 0.456, 0.567, 0.678, 0.789, 0.900, 1.011, 1.123,
      1.234, 1.345;
  ct << 1, 0, 0, 1;
  MiniCheetahKin kin_data(js, ct);
  std::cout << kin_data.get_joint_state();
  kin_data.compute_kinematics();
  std::cout << kin_data.get_kin_pos(FL);
  EXPECT_EQ(kin_data.get_contact(FR), true);
  EXPECT_EQ(kin_data.get_contact(FL), false);
  EXPECT_EQ(kin_data.get_contact(HL), false);
  EXPECT_EQ(kin_data.get_contact(HR), true);
  std::cout << kin_data.get_J(FR);
}