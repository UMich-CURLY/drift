#include <gtest/gtest.h>
#include "measurement/imu.h"


TEST(ImuMeasurementTest, CtorType) {
  ImuMeasurement<double> imu_data;
  EXPECT_EQ(imu_data.get_type(), 1);
}

TEST(ImuMeasurementTest, QuaternionSetGet) {
  ImuMeasurement<double> imu_data;
  imu_data.set_quaternion(0.11, 0.12, 0.22, 0.44);
  EXPECT_EQ(imu_data.get_quaternion_x(), 0.11);
  EXPECT_EQ(imu_data.get_quaternion_y(), 0.12);
  EXPECT_EQ(imu_data.get_quaternion_z(), 0.22);
  EXPECT_EQ(imu_data.get_quaternion_w(), 0.44);
}

TEST(ImuMeasurementTest, QuaternionToRotMat) {
  ImuMeasurement<float> imu_data;
  float angle = M_PI / 4;
  float sinA = std::sin(angle / 2);
  float cosA = std::cos(angle / 2);

  Eigen::Quaternionf q;
  q.x() = 0 * sinA;
  q.y() = 1 * sinA;
  q.z() = 0 * sinA;
  q.w() = cosA;

  imu_data.set_quaternion(q.x(), q.y(), q.z(), q.w());

  Eigen::Matrix<double, 3, 3> rotmattest;
  rotmattest << 0.707107, 0, 0.707107, 0, 1, 0, -0.707107, 0, 0.707107;

  EXPECT_NEAR(imu_data.get_rotation_matrix()(0, 0), rotmattest(0, 0), 1e-5);
}
