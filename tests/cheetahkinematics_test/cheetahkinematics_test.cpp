#include <gtest/gtest.h>
#include <iostream>
#include "drift/kinematics/mini_cheetah_kinematics.h"

#define tol3 1e-9

using namespace mini_cheetah_kinematics;
using namespace measurement;

TEST(cheetahkinematicstest, DefaultCtor) {
  kinematics::MiniCheetahKinematics kin_data;
  EXPECT_EQ(kin_data.get_type(), 2);
  EXPECT_EQ(kin_data.get_type(), LEGGED_KINEMATICS);

  for (size_t i = 0; i < 3; i++) {
    EXPECT_EQ(kin_data.get_kin_pos(1)(i), 0);
  }

  EXPECT_EQ(kin_data.get_J(1).rows(), 3);
  EXPECT_EQ(kin_data.get_J(1).cols(), 3);
  for (size_t i = 0; i < 9; i++) {
    EXPECT_EQ(kin_data.get_J(1)(i), 0);
  }

  for (size_t i = 0; i < 4; i++) {
    EXPECT_EQ(kin_data.get_contact(i), 0);
  }

  for (size_t i = 0; i < 12; i++) {
    EXPECT_EQ(kin_data.get_joint_state(i), 0);
  }
}

TEST(cheetahkinematicstest, JointStateSetGet) {
  kinematics::MiniCheetahKinematics kin_data;
  Eigen::Matrix<double, 12, 1> v;
  v << 0.123, 0.234, 0.345, 0.456, 0.567, 0.678, 0.789, 0.900, 1.011, 1.123,
      1.234, 1.345;
  kin_data.set_joint_state(v);
  for (size_t i = 0; i < 12; i++) {
    EXPECT_EQ(kin_data.get_joint_state(i), v[i]);
  }
  EXPECT_EQ(kin_data.get_num_legs(), 4);
}

TEST(cheetahkinematicstest, ContactsSetGet) {
  kinematics::MiniCheetahKinematics kin_data;
  Eigen::Matrix<bool, 4, 1> c;
  c << 0, 1, 1, 0;
  kin_data.set_contact(c);
  for (size_t i = 0; i < 4; i++) {
    EXPECT_EQ(kin_data.get_contact(i), c[i]);
  }
}

TEST(cheetahkinematicstest, OverloadCtor) {
  Eigen::Matrix<double, 12, 1> js;
  Eigen::Matrix<double, 12, 1> js_vel;
  Eigen::Matrix<bool, 4, 1> ct;
  js << 0.123, 0.234, 0.345, 0.456, 0.567, 0.678, 0.789, 0.900, 1.011, 1.123,
      1.234, 1.345;
  js_vel << 0, 0, 0, 0, 0, 0., 0, 0, 0, 0, 0, 0;
  ct << 1, 0, 0, 1;
  kinematics::MiniCheetahKinematics kin_data(js, js_vel, ct);
  EXPECT_EQ(kin_data.get_contact(FR), true);
  EXPECT_EQ(kin_data.get_contact(FL), false);
  EXPECT_EQ(kin_data.get_contact(HR), false);
  EXPECT_EQ(kin_data.get_contact(HL), true);
  kin_data.ComputeKinematics();
  Eigen::Matrix<double, 3, 3> JpFR = kin_data.get_J(FR);
  Eigen::Matrix<double, 3, 3> JpFL = kin_data.get_J(FL);
  Eigen::Matrix<double, 3, 3> JpHR = kin_data.get_J(HR);
  Eigen::Matrix<double, 3, 3> JpHL = kin_data.get_J(HL);
  js << 0.123, 0.234, 0.345, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  kin_data.set_joint_state(js);
  kin_data.ComputeKinematics();
  EXPECT_EQ(kin_data.get_J(FR), JpFR);
  EXPECT_NE(kin_data.get_J(FL), JpFL);
  EXPECT_NE(kin_data.get_J(HR), JpHR);
  EXPECT_NE(kin_data.get_J(HL), JpHL);
}

TEST(cheetahkinematicstest, Position) {
  Eigen::Matrix<double, 12, 1> js;
  Eigen::Matrix<double, 12, 1> js_vel;
  Eigen::Matrix<bool, 4, 1> ct;
  js << 0.123, 0.234, 0.345, 0.456, 0.567, 0.678, 0.789, 0.900, 1.011, 1.123,
      1.234, 1.345;
  js_vel << 0, 0, 0, 0, 0, 0., 0, 0, 0, 0, 0, 0;
  ct << 0, 1, 0, 1;
  kinematics::MiniCheetahKinematics kin_data(js, js_vel, ct);
  EXPECT_EQ(kin_data.get_contact(FR), false);
  EXPECT_EQ(kin_data.get_contact(FL), true);
  EXPECT_EQ(kin_data.get_contact(HR), false);
  EXPECT_EQ(kin_data.get_contact(HL), true);
  kin_data.ComputeKinematics();
  Eigen::Matrix<double, 3, 1> pFR = kin_data.get_kin_pos(FR);
  Eigen::Matrix<double, 3, 1> pFL = kin_data.get_kin_pos(FL);
  Eigen::Matrix<double, 3, 1> pHR = kin_data.get_kin_pos(HR);
  Eigen::Matrix<double, 3, 1> pHL = kin_data.get_kin_pos(HL);
  js << 0.123, 0.234, 0.345, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
  kin_data.set_joint_state(js);
  kin_data.ComputeKinematics();
  EXPECT_EQ(kin_data.get_kin_pos(FR), pFR);
  EXPECT_NE(kin_data.get_kin_pos(FL), pFL);
  EXPECT_NE(kin_data.get_kin_pos(HR), pHR);
  EXPECT_NE(kin_data.get_kin_pos(HL), pHL);
}