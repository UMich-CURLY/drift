#include <gtest/gtest.h>
#include "drift/measurement/imu.h"

#define tol1 1e-6

template<typename T>
void compare_rot_mat(Eigen::Matrix<T, 3, 3> imu, Eigen::Matrix<T, 3, 3> test);

template<typename T>
Eigen::Quaternion<T> rotaxis2quat(T a, T b_x, T b_y, T b_z);

using namespace measurement;

TEST(ImuMeasurementTest, Ctor) {
  ImuMeasurement<double> imu_data;
  EXPECT_EQ(imu_data.get_type(), 1);
  EXPECT_EQ(imu_data.get_type(), IMU);
  EXPECT_EQ(imu_data.get_quaternion().w(), 1);
  EXPECT_EQ(imu_data.get_quaternion().x(), 0);
  compare_rot_mat<double>(imu_data.get_rotation_matrix(),
                          Eigen::Matrix3d::Identity());
}


TEST(ImuMeasurementTest, QuaternionSetGetBasic) {
  ImuMeasurement<double> imu_data;

  Eigen::Quaterniond q = rotaxis2quat<double>(M_PI / 4, M_PI / 2, 0, M_PI / 2);
  imu_data.set_quaternion(q.w(), q.x(), q.y(), q.z());

  // comparison values obtained with
  // https://www.andre-gaschler.com/rotationconverter/
  EXPECT_NEAR(imu_data.get_quaternion().x(), 0, tol1);
  EXPECT_NEAR(imu_data.get_quaternion().y(), 0.3826834, tol1);
  EXPECT_NEAR(imu_data.get_quaternion().z(), 0, tol1);
  EXPECT_NEAR(imu_data.get_quaternion().w(), 0.9238795, tol1);
}

TEST(ImuMeasurementTest, QuaternionToRotMat1) {
  ImuMeasurement<float> imu_data;

  Eigen::Quaternionf q = rotaxis2quat<float>(M_PI / 4, M_PI / 2, 0, M_PI / 2);

  imu_data.set_quaternion(q.w(), q.x(), q.y(), q.z());

  // rotmattest created using values from
  // https://www.andre-gaschler.com/rotationconverter/
  Eigen::Matrix<float, 3, 3> rotmattest;
  rotmattest << 0.707107, 0, 0.707107, 0, 1, 0, -0.707107, 0, 0.707107;
  compare_rot_mat<float>(imu_data.get_rotation_matrix(), rotmattest);
}

TEST(ImuMeasurementTest, QuaternionToRotMat2) {
  ImuMeasurement<float> imu_data;

  Eigen::Quaternionf q = rotaxis2quat<float>(M_PI / 3, M_PI / 2, M_PI / 2, 0);

  imu_data.set_quaternion(q.w(), q.x(), q.y(), q.z());

  Eigen::Matrix<float, 3, 3> rotmattest;
  rotmattest << 0.5, -0.8660254, 0, 0.8660254, 0.5, 0, 0, 0, 1;
  compare_rot_mat<float>(imu_data.get_rotation_matrix(), rotmattest);
}

TEST(ImuMeasurementTest, QuaternionToRotMat3) {
  ImuMeasurement<double> imu_data;

  Eigen::Quaterniond q
      = rotaxis2quat<double>(M_PI / 6, M_PI / 4, M_PI / 2, M_PI / 4);

  imu_data.set_quaternion(q.w(), q.x(), q.y(), q.z());

  Eigen::Matrix<double, 3, 3> rotmattest;
  rotmattest << 0.9330127, -0.3535534, 0.0669873, 0.3535534, 0.8660254,
      -0.3535534, 0.0669873, 0.3535534, 0.9330127;
  compare_rot_mat<double>(imu_data.get_rotation_matrix(), rotmattest);
}

TEST(ImuMeasurementTest, QuaternionRepresentationInvariant) {
  ImuMeasurement<double> imu_data;

  EXPECT_THROW(imu_data.set_quaternion(0.123, 0.456, 0.789, 0.012),
               std::invalid_argument);
  EXPECT_THROW(imu_data.set_quaternion(1, 2, 3, 4), std::invalid_argument);
  EXPECT_THROW(imu_data.set_quaternion(0.2, 0.2, 0.2, 0.2),
               std::invalid_argument);
  Eigen::Quaterniond q
      = rotaxis2quat<double>(M_PI / 6, M_PI / 4, M_PI / 2, M_PI / 4);

  EXPECT_NO_THROW(imu_data.set_quaternion(q.w(), q.x(), q.y(), q.z()));
}

TEST(ImuMeasurementTest, AngularVelocitySetGetBasic) {
  ImuMeasurement<double> imu_data;
  imu_data.set_angular_velocity(1, 2, 3);
  EXPECT_EQ(imu_data.get_angular_velocity().x(), 1);
  EXPECT_EQ(imu_data.get_angular_velocity().z(), 3);
}

TEST(ImuMeasurementTest, LinearAccelerationSetGetBasic) {
  ImuMeasurement<double> imu_data;
  imu_data.set_lin_acc(1, 2, 3);
  EXPECT_EQ(imu_data.get_lin_acc().x(), 1);
  EXPECT_EQ(imu_data.get_lin_acc().z(), 3);
}

// Helper Functions
template<typename T>
void compare_rot_mat(Eigen::Matrix<T, 3, 3> imu, Eigen::Matrix<T, 3, 3> test) {
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      EXPECT_NEAR(imu(r, c), test(r, c), tol1);
    }
  }
}

// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Intuition
template<typename T>
Eigen::Quaternion<T> rotaxis2quat(T a, T b_x, T b_y, T b_z) {
  T sinA = std::sin(a / 2);
  T cosA = std::cos(a / 2);
  Eigen::Quaternion<T> q;
  q.x() = std::cos(b_x) * sinA;
  q.y() = std::cos(b_y) * sinA;
  q.z() = std::cos(b_z) * sinA;
  q.w() = cosA;

  return q;
}
