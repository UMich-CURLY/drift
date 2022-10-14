#include <gtest/gtest.h>
#include "measurement/imu.h"

void compare_rotation_matrix(Eigen::Matrix3d imu, Eigen::Matrix3d test);

template<typename T>
Eigen::Quaternion<T> rotaxis2quaternion(T a, T b_x, T b_y, T b_z);

TEST(ImuMeasurementTest, CtorType) {
  ImuMeasurement<double> imu_data;
  EXPECT_EQ(imu_data.get_type(), 1);
}

TEST(ImuMeasurementTest, QuaternionSetGetBasic) {
  ImuMeasurement<double> imu_data;

  Eigen::Quaterniond q
      = rotaxis2quaternion<double>(M_PI / 4, M_PI / 2, 0, M_PI / 2);
  imu_data.set_quaternion(q.x(), q.y(), q.z(), q.w());

  EXPECT_NEAR(imu_data.get_quaternion_x(), 0, 1e-5);
  EXPECT_NEAR(imu_data.get_quaternion_y(), 0.3826834, 1e-5);
  EXPECT_NEAR(imu_data.get_quaternion_z(), 0, 1e-5);
  EXPECT_NEAR(imu_data.get_quaternion_w(), 0.9238795, 1e-5);
}

TEST(ImuMeasurementTest, QuaternionToRotMat1) {
  ImuMeasurement<float> imu_data;

  Eigen::Quaternionf q
      = rotaxis2quaternion<float>(M_PI / 4, M_PI / 2, 0, M_PI / 2);

  imu_data.set_quaternion(q.x(), q.y(), q.z(), q.w());

  // rotmattest created using https://www.andre-gaschler.com/rotationconverter/
  Eigen::Matrix<double, 3, 3> rotmattest;
  rotmattest << 0.707107, 0, 0.707107, 0, 1, 0, -0.707107, 0, 0.707107;
  compare_rotation_matrix(imu_data.get_rotation_matrix(), rotmattest);
}

TEST(ImuMeasurementTest, QuaternionToRotMat2) {
  ImuMeasurement<float> imu_data;

  Eigen::Quaternionf q
      = rotaxis2quaternion<float>(M_PI / 3, M_PI / 2, M_PI / 2, 0);

  imu_data.set_quaternion(q.x(), q.y(), q.z(), q.w());

  Eigen::Matrix<double, 3, 3> rotmattest;
  rotmattest << 0.5, -0.8660254, 0, 0.8660254, 0.5, 0, 0, 0, 1;
  compare_rotation_matrix(imu_data.get_rotation_matrix(), rotmattest);
}

TEST(ImuMeasurementTest, QuaternionToRotMat3) {
  ImuMeasurement<double> imu_data;

  Eigen::Quaterniond q
      = rotaxis2quaternion<double>(M_PI / 6, M_PI / 4, M_PI / 2, M_PI / 4);

  imu_data.set_quaternion(q.x(), q.y(), q.z(), q.w());

  Eigen::Matrix<double, 3, 3> rotmattest;
  rotmattest << 0.9330127, -0.3535534, 0.0669873, 0.3535534, 0.8660254,
      -0.3535534, 0.0669873, 0.3535534, 0.9330127;
  compare_rotation_matrix(imu_data.get_rotation_matrix(), rotmattest);
}

void compare_rotation_matrix(Eigen::Matrix3d imu, Eigen::Matrix3d test) {
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      EXPECT_NEAR(imu(r, c), test(r, c), 1e-5);
    }
  }
}


/**
 * https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Intuition
 *
 * @param[in] a: alpha coefficient.
 * @param[in] b_x: beta x coefficient.
 * @param[in] b_y: beta y coefficient.
 * @param[in] b_z: beta z coefficient.
 */
template<typename T>
Eigen::Quaternion<T> rotaxis2quaternion(T a, T b_x, T b_y, T b_z) {
  T sinA = std::sin(a / 2);
  T cosA = std::cos(a / 2);
  Eigen::Quaternion<T> q;
  q.x() = std::cos(b_x) * sinA;
  q.y() = std::cos(b_y) * sinA;
  q.z() = std::cos(b_z) * sinA;
  q.w() = cosA;
  return q;
}
