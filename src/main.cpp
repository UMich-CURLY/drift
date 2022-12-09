
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
#include "filter/noise_params.h"
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

// #define LCM_MULTICAST_URL "udpm://239.255.76.67:7667?ttl=2"
using namespace std::chrono;


int main(int argc, char** argv) {
  Eigen::Matrix<double, 5, 5> m;
  m << 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
      1;

  Eigen::Matrix<double, 6, 1> imu;
  imu << 0, 0, 0, 0, 0, 9.81;

  double dt = 1.0;
  double T = 3 * dt;

  Eigen::Vector3d measured_velocity;
  measured_velocity << 0, 0, 0;

  Eigen::Matrix3d measured_velocity_covariance;
  measured_velocity_covariance << 0, 0, 0, 0, 0, 0, 0, 0, 0;

  NoiseParams params;
  double temp_param = 0;
  params.set_gyroscope_noise(temp_param);
  params.set_accelerometer_noise(temp_param);
  params.set_gyroscope_bias_noise(temp_param);
  params.set_accelerometer_bias_noise(temp_param);
  params.set_augment_noise("contact", temp_param);

  inekf::ErrorType error_type = RightInvariant;
  StateEstimator state_estimator(params, error_type);
  RobotState state(m);
  state_estimator.set_state(state);
  std::cout << "Before: " << std::endl;
  std::cout << state_estimator.get_state().get_X() << std::endl;

  std::queue<ImuMeasurement<double>> imu_data_buffer;
  std::shared_ptr<std::queue<ImuMeasurement<double>>> imu_data_buffer_ptr
      = std::make_shared<std::queue<ImuMeasurement<double>>>(imu_data_buffer);

  std::queue<VelocityMeasurement<double>> velocity_data_buffer;
  std::shared_ptr<std::queue<VelocityMeasurement<double>>>
      velocity_data_buffer_ptr
      = std::make_shared<std::queue<VelocityMeasurement<double>>>(
          velocity_data_buffer);


  // // Set measurements:
  VelocityMeasurement<double> velocity_measurement_1;
  ImuMeasurement<double> imu_measurement_1;

  velocity_measurement_1.set_velocity(0, 0, 0);
  velocity_measurement_1.set_time(dt);
  imu_measurement_1.set_ang_vel(0, 0, 0);
  imu_measurement_1.set_lin_acc(-1, 0, 9.81);
  imu_measurement_1.set_time(dt);
  imu_data_buffer_ptr.get()->push(imu_measurement_1);
  velocity_data_buffer_ptr.get()->push(velocity_measurement_1);

  VelocityMeasurement<double> velocity_measurement_2;
  ImuMeasurement<double> imu_measurement_2;
  velocity_measurement_2.set_velocity(0, 0, 0);
  velocity_measurement_2.set_time(dt * 2);
  imu_measurement_2.set_ang_vel(0, 0, 90.0 / 180.0 * M_PI);
  imu_measurement_2.set_lin_acc(0, 0, 9.81);
  imu_measurement_2.set_time(dt * 2);
  imu_data_buffer_ptr.get()->push(imu_measurement_2);
  velocity_data_buffer_ptr.get()->push(velocity_measurement_2);

  VelocityMeasurement<double> velocity_measurement_3;
  ImuMeasurement<double> imu_measurement_3;
  velocity_measurement_3.set_velocity(1, 0, 0);
  velocity_measurement_3.set_time(dt * 3);
  imu_measurement_3.set_ang_vel(0, 0, 0);
  imu_measurement_3.set_lin_acc(1, 0, 9.81);
  imu_measurement_3.set_time(dt * 3);
  imu_data_buffer_ptr.get()->push(imu_measurement_3);
  velocity_data_buffer_ptr.get()->push(velocity_measurement_3);


  state_estimator.add_imu_propagation(imu_data_buffer_ptr, false);
  state_estimator.add_velocity_correction(velocity_data_buffer_ptr,
                                          measured_velocity_covariance);
  for (int i = 0; i < 3; i++) {
    state_estimator.run();
    std::cout << "After: " << std::endl;
    std::cout << state_estimator.get_state().get_X() << std::endl;
    std::cout << "------------------------------------------" << std::endl;
  }
}