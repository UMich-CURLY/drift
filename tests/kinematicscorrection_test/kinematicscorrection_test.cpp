
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
#include "filter/inekf/correction/legged_kinematics_correction.h"
#include "filter/inekf/correction/velocity_correction.h"
#include "filter/inekf/propagation/imu_propagation.h"
#include "measurement/imu.h"
#include "measurement/kinematics.h"
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


TEST(LeggedKinematicsCorrection, ImuPropVelCorr) {
  // Initialize a state matrix m
  Eigen::Matrix<double, 5, 5> m;
  m << 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
      1;

  // Initialize time step
  double dt = 1.0;
  double T = 3 * dt;

  inekf::ErrorType error_type = RightInvariant;

  StateEstimator state_estimator(error_type);

  IMUQueue imu_data_buffer;
  IMUQueuePtr imu_data_buffer_ptr = std::make_shared<IMUQueue>(imu_data_buffer);
  KinematicsQueue kinematics_data_buffer;
  LeggedKinematicsQueuePtr kinematics_data_buffer_ptr
      = std::make_shared<KinematicsQueue>(kinematics_data_buffer);

  imu_measurement_0.set_ang_vel(0, 0, 0);
  imu_measurement_0.set_lin_acc(0, 0, 9.8);
  imu_measurement_0.set_time(0);
  imu_data_buffer_ptr.get()->push(
      std::make_shared<ImuMeasurement<double>>(imu_measurement_0));

  ImuMeasurement<double> imu_measurement_1;
  imu_measurement_1.set_ang_vel(0, 0, 0);
  imu_measurement_1.set_lin_acc(-1, 0, 9.8);
  imu_measurement_1.set_time(dt);
  imu_data_buffer_ptr.get()->push(
      std::make_shared<ImuMeasurement<double>>(imu_measurement_1));

  ImuMeasurement<double> imu_measurement_2;
  imu_measurement_2.set_ang_vel(0, 0, 90.0 / 180.0 * M_PI);
  imu_measurement_2.set_lin_acc(0, 0, 9.8);
  imu_measurement_2.set_time(dt * 2);
  imu_data_buffer_ptr.get()->push(
      std::make_shared<ImuMeasurement<double>>(imu_measurement_2));

  ImuMeasurement<double> imu_measurement_3;
  imu_measurement_3.set_ang_vel(0, 0, 0);
  imu_measurement_3.set_lin_acc(1, 0, 9.8);
  imu_measurement_3.set_time(dt * 3);
  imu_data_buffer_ptr.get()->push(
      std::make_shared<ImuMeasurement<double>>(imu_measurement_3));

  std::shared_ptr<std::mutex> imu_buffer_mutex_ptr(new std::mutex);
  std::shared_ptr<std::mutex> kinematics_buffer_mutex_ptr(new std::mutex);


  // Add propagation and correction methods
  state_estimator.add_imu_propagation(imu_data_buffer_ptr,
                                      imu_buffer_mutex_ptr);
  state_estimator.add_kinematics_correction(kinematics_data_buffer_ptr,
                                            kinematics_buffer_mutex_ptr);
}