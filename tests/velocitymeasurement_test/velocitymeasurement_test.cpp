#include <gtest/gtest.h>
#include "measurement/velocity.h"

TEST(VelocityMeasurementTest, Ctor) {
  VelocityMeasurement<double> velocity_data;
  EXPECT_EQ(velocity_data.get_type(), 3);
  EXPECT_EQ(velocity_data.get_type(), VELOCITY);
  EXPECT_EQ(velocity_data.get_velocity().x, 0);
}

/*
TEST(ContactMeasurementTest, QuaternionSetGetBasic) {
  ContactMeasurement<double> velocity_data;

  Eigen::Quaterniond q = rotaxis2quat<double>(M_PI / 4, M_PI / 2, 0, M_PI / 2);
  velocity_data.set_quaternion(q.w(), q.x(), q.y(), q.z());

  double tol = 1e-5;
  // comparison values obtained with
  // https://www.andre-gaschler.com/rotationconverter/
  EXPECT_NEAR(velocity_data.get_quaternion().x, 0, tol);
  EXPECT_NEAR(velocity_data.get_quaternion().y, 0.3826834, tol);
  EXPECT_NEAR(velocity_data.get_quaternion().z, 0, tol);
  EXPECT_NEAR(velocity_data.get_quaternion().w, 0.9238795, tol);
}

TEST(ContactMeasurementTest, QuaternionToRotMat1) {
  ContactMeasurement<float> velocity_data;

  Eigen::Quaternionf q = rotaxis2quat<float>(M_PI / 4, M_PI / 2, 0, M_PI / 2);

  velocity_data.set_quaternion(q.w(), q.x(), q.y(), q.z());

  // rotmattest created using values from
  // https://www.andre-gaschler.com/rotationconverter/
  Eigen::Matrix<double, 3, 3> rotmattest;
  rotmattest << 0.707107, 0, 0.707107, 0, 1, 0, -0.707107, 0, 0.707107;
  compare_rot_mat(velocity_data.get_rotation_matrix(), rotmattest);
}

TEST(ContactMeasurementTest, QuaternionToRotMat2) {
  ContactMeasurement<float> velocity_data;

  Eigen::Quaternionf q = rotaxis2quat<float>(M_PI / 3, M_PI / 2, M_PI / 2, 0);

  velocity_data.set_quaternion(q.w(), q.x(), q.y(), q.z());

  Eigen::Matrix<double, 3, 3> rotmattest;
  rotmattest << 0.5, -0.8660254, 0, 0.8660254, 0.5, 0, 0, 0, 1;
  compare_rot_mat(velocity_data.get_rotation_matrix(), rotmattest);
}

TEST(ContactMeasurementTest, QuaternionToRotMat3) {
  ContactMeasurement<double> velocity_data;

  Eigen::Quaterniond q
      = rotaxis2quat<double>(M_PI / 6, M_PI / 4, M_PI / 2, M_PI / 4);

  velocity_data.set_quaternion(q.w(), q.x(), q.y(), q.z());

  Eigen::Matrix<double, 3, 3> rotmattest;
  rotmattest << 0.9330127, -0.3535534, 0.0669873, 0.3535534, 0.8660254,
      -0.3535534, 0.0669873, 0.3535534, 0.9330127;
  compare_rot_mat(velocity_data.get_rotation_matrix(), rotmattest);
}

TEST(ContactMeasurementTest, QuaternionRepresentationInvariant) {
  ContactMeasurement<double> velocity_data;

  EXPECT_THROW(velocity_data.set_quaternion(0.123, 0.456, 0.789, 0.012),
               std::invalid_argument);
  EXPECT_THROW(velocity_data.set_quaternion(1, 2, 3, 4), std::invalid_argument);
  EXPECT_THROW(velocity_data.set_quaternion(0.2, 0.2, 0.2, 0.2),
               std::invalid_argument);
  Eigen::Quaterniond q
      = rotaxis2quat<double>(M_PI / 6, M_PI / 4, M_PI / 2, M_PI / 4);

  EXPECT_NO_THROW(velocity_data.set_quaternion(q.w(), q.x(), q.y(), q.z()));
}

TEST(ContactMeasurementTest, AngularVelocitySetGetBasic) {
  ContactMeasurement<double> velocity_data;
  velocity_data.set_ang_vel(1, 2, 3);
  EXPECT_EQ(velocity_data.get_ang_vel().x, 1);
  EXPECT_EQ(velocity_data.get_ang_vel().z, 3);
}

TEST(ContactMeasurementTest, LinearAccelerationSetGetBasic) {
  ContactMeasurement<double> velocity_data;
  velocity_data.set_lin_acc(1, 2, 3);
  EXPECT_EQ(velocity_data.get_lin_acc().x, 1);
  EXPECT_EQ(velocity_data.get_lin_acc().z, 3);
}

// Helper Functions

void compare_rot_mat(Eigen::Matrix3d imu, Eigen::Matrix3d test) {
  double tol = 1e-5;
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      EXPECT_NEAR(imu(r, c), test(r, c), tol);
    }
  }
}

//
https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Intuition
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
*/