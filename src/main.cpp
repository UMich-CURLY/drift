
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
#include "state/robot_state.h"

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

  double dt = 0.01;

  Eigen::Vector3d measured_velocity;
  measured_velocity << 1, 0, 0;

  Eigen::Matrix3d measured_velocity_covariance;
  measured_velocity_covariance << 0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01;

  NoiseParams params;
  double temp_param = 0;
  params.set_gyroscope_noise(temp_param);
  params.set_accelerometer_noise(temp_param);
  params.set_gyroscope_bias_noise(temp_param);
  params.set_accelerometer_bias_noise(temp_param);
  // params.setContactNoise(temp_param);

  //   std::queue<Eigen::MatrixXd<double, 6, 1>> imu_data_buffer;
  //   std::shared_ptr<std::queue<Eigen::MatrixXd<double, 6, 1>>>
  //   imu_data_buffer_ptr
  //       = std::make_shared<std::queue<Eigen::MatrixXd<double, 6, 1>>>(
  //           imu_data_buffer);

  //   std::queue<Eigen::Vector3d> velocity_data_buffer;
  //   std::shared_ptr<std::queue<Eigen::Vector3d>> velocity_data_buffer_ptr
  //       =
  //       std::make_shared<std::queue<Eigen::Vector3d>>(velocity_data_buffer);

  //   RobotState state(m);
  //   inekf::Propagation<Eigen::MatrixXd<double, 6, 1>> propagation(
  //       imu_data_buffer_ptr, params, inekf::ErrorType::RightInvariant);
  //   inekf::VelocityCorrection<Eigen::Vector3d> correction(
  //       velocity_data_buffer_ptr, inekf::ErrorType::RightInvariant,
  //       measured_velocity_covariance);


  //   propagation.Propagate(state);
  //   correction.Correct(state);
  //   std::cout << state.getX() << std::endl;
}