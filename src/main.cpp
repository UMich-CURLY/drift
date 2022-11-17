
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
#include <thread>
// #include "utils/cheetah_data_t.hpp"
// #include "communication/lcm_handler.hpp"
#include "inekf/inekf_correct.h"
#include "inekf/inekf_propagate.h"
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
    m << 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;

    Eigen::Matrix<double, 6, 1> imu;
    imu << 0, 0, 0, 0, 0, 9.81;

    double dt = 0.01;

    Eigen::Vector3d measured_velocity;
    measured_velocity << 1, 0, 0;

    Eigen::Matrix3d measured_velocity_covariance;
    measured_velocity_covariance << 0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01;

    inekf::NoiseParams params;
    double temp_param = 0;
    params.setGyroscopeNoise(temp_param);
    params.setAccelerometerNoise(temp_param);
    params.setGyroscopeBiasNoise(temp_param);
    params.setAccelerometerBiasNoise(temp_param);
    params.setContactNoise(temp_param);

    sensor_data_t sensor_data_buffer;
    std::shared_ptr<sensor_data_t> sensor_data_buffer_ptr = std::make_shared<sensor_data_t>(sensor_data_buffer);

    inekf::RobotState state(m);
    inekf::Propagation propagation(sensor_data_buffer_ptr, params, inekf::ErrorType::RightInvariant);
    inekf::VelocityCorrection correction(sensor_data_buffer_ptr, inekf::ErrorType::RightInvariant);


    propagation.Propagate(imu, dt, state);
    correction.Correct(measured_velocity, measured_velocity_covariance, state);
    std::cout << state.getX() << std::endl;
}