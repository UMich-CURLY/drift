
// STL
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include "filter/base_correction.h"
#include "filter/base_propagation.h"
#include "filter/inekf/correction/velocity_correction.h"
#include "filter/inekf/propagation/imu_propagation.h"
#include "measurement/imu.h"
#include "measurement/velocity.h"
#include "state/robot_state.h"
#include "state_estimator.h"

// Boost
#include <boost/algorithm/string.hpp>
// Threading
#include <boost/circular_buffer.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

// Testing
#include <gtest/gtest.h>

using namespace measurement;

TEST(VelocityCorrection, ImuPropVelCorr) {
  // Initialize a state matrix m
  Eigen::Matrix<double, 5, 5> m;
  m << 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
      1;

  // Initialize time step
  double dt = 1.0;
  double T = 3 * dt;

  Eigen::Vector3d measured_velocity;
  measured_velocity << 0, 0, 0;

  Eigen::Matrix3d measured_velocity_covariance;
  measured_velocity_covariance << 0, 0, 0, 0, 0, 0, 0, 0, 0;

  inekf::ErrorType error_type = LeftInvariant;

  StateEstimator state_estimator(error_type);

  IMUQueue imu_data_buffer;
  IMUQueuePtr imu_data_buffer_ptr = std::make_shared<IMUQueue>(imu_data_buffer);

  VelocityQueue velocity_data_buffer;
  VelocityQueuePtr velocity_data_buffer_ptr
      = std::make_shared<VelocityQueue>(velocity_data_buffer);

  // Set measurements:
  VelocityMeasurement<double> velocity_measurement_0;
  ImuMeasurement<double> imu_measurement_0;

  velocity_measurement_0.set_velocity(1, 0, 0);
  velocity_measurement_0.set_time(0);
  imu_measurement_0.set_ang_vel(0, 0, 0);
  imu_measurement_0.set_lin_acc(0, 0, 9.81);
  imu_measurement_0.set_time(0);
  imu_data_buffer_ptr.get()->push(
      std::make_shared<ImuMeasurement<double>>(imu_measurement_0));
  velocity_data_buffer_ptr.get()->push(
      std::make_shared<VelocityMeasurement<double>>(velocity_measurement_0));

  VelocityMeasurement<double> velocity_measurement_1;
  ImuMeasurement<double> imu_measurement_1;

  velocity_measurement_1.set_velocity(0, 0, 0);
  velocity_measurement_1.set_time(dt);
  imu_measurement_1.set_ang_vel(0, 0, 0);
  imu_measurement_1.set_lin_acc(-1, 0, 9.81);
  imu_measurement_1.set_time(dt);
  imu_data_buffer_ptr.get()->push(
      std::make_shared<ImuMeasurement<double>>(imu_measurement_1));
  velocity_data_buffer_ptr.get()->push(
      std::make_shared<VelocityMeasurement<double>>(velocity_measurement_1));

  VelocityMeasurement<double> velocity_measurement_2;
  ImuMeasurement<double> imu_measurement_2;
  velocity_measurement_2.set_velocity(0, 0, 0);
  velocity_measurement_2.set_time(dt * 2);
  imu_measurement_2.set_ang_vel(0, 0, 90.0 / 180.0 * M_PI);
  imu_measurement_2.set_lin_acc(0, 0, 9.81);
  imu_measurement_2.set_time(dt * 2);
  imu_data_buffer_ptr.get()->push(
      std::make_shared<ImuMeasurement<double>>(imu_measurement_2));
  velocity_data_buffer_ptr.get()->push(
      std::make_shared<VelocityMeasurement<double>>(velocity_measurement_2));

  VelocityMeasurement<double> velocity_measurement_3;
  ImuMeasurement<double> imu_measurement_3;
  velocity_measurement_3.set_velocity(1, 0, 0);
  velocity_measurement_3.set_time(dt * 3);
  imu_measurement_3.set_ang_vel(0, 0, 0);
  imu_measurement_3.set_lin_acc(1, 0, 9.81);
  imu_measurement_3.set_time(dt * 3);
  imu_data_buffer_ptr.get()->push(
      std::make_shared<ImuMeasurement<double>>(imu_measurement_3));
  velocity_data_buffer_ptr.get()->push(
      std::make_shared<VelocityMeasurement<double>>(velocity_measurement_3));

  std::shared_ptr<std::mutex> imu_buffer_mutex_ptr(new std::mutex);
  std::shared_ptr<std::mutex> velocity_buffer_mutex_ptr(new std::mutex);

  // Add propagation and correction methods
  state_estimator.add_imu_propagation(
      imu_data_buffer_ptr, imu_buffer_mutex_ptr,
      "../config/filter/inekf/propagation/velocitycorrection_test.yaml");
  state_estimator.add_velocity_correction(
      velocity_data_buffer_ptr, velocity_buffer_mutex_ptr,
      "../config/filter/inekf/correction/velocitycorrection_test.yaml");

  std::vector<Eigen::Matrix<double, 5, 5>> expect_X;
  Eigen::Matrix<double, 5, 5> X = Eigen::Matrix<double, 5, 5>::Identity();

  X(0, 3) = 1;
  expect_X.push_back(X);

  X(0, 3) = 0;
  X(0, 4) = 0.5;
  expect_X.push_back(X);

  Eigen::Matrix2d R;
  R << 0, -1, 1, 0;
  X.block<2, 2>(0, 0) = R;
  expect_X.push_back(X);

  X(1, 3) = 1;
  X(1, 4) = 0.5;
  expect_X.push_back(X);

  // Set initial state:
  state_estimator.InitState();
  auto init_state = state_estimator.get_state().get_X();
  // Check initial state value:
  for (int j = 0; j < 5; j++) {
    for (int k = 0; k < 5; k++) {
      EXPECT_NEAR(init_state(j, k), expect_X[0](j, k), 1e-6);
    }
  }

  for (int i = 1; i < 4; i++) {
    // Check propagation and correction:
    if (state_estimator.is_enabled()) {
      state_estimator.RunOnce();
      std::cout << "After Correction: " << std::endl;
      std::cout << state_estimator.get_state().get_X() << std::endl;
      auto est_state = state_estimator.get_state().get_X();
      for (int j = 0; j < 5; j++) {
        for (int k = 0; k < 5; k++) {
          EXPECT_NEAR(est_state(j, k), expect_X[i](j, k), 1e-6);
        }
      }
      std::cout << "------------------------------------------" << std::endl;
    } else {
      if (state_estimator.BiasInitialized()) {
        state_estimator.InitState();
        state_estimator.EnableFilter();
      } else {
        state_estimator.InitBias();
      }
    }
  }
}