
// STL
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <thread>
#include <chrono>
#include <fstream>
#include <string>
#include <memory>
#include <iostream>
#include <chrono>
// #include "utils/cheetah_data_t.hpp"
// #include "communication/lcm_handler.hpp"
#include "inekf/inekf_correct.h"
#include "inekf/inekf_propagate.h"
#include "inekf/inekf_obj.h"
#include "state/robot_state.h"

// Boost
#include <boost/algorithm/string.hpp>
// Threading
#include <boost/thread/condition.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>

// #define LCM_MULTICAST_URL "udpm://239.255.76.67:7667?ttl=2"
using namespace std::chrono;


int main(int argc, char **argv)
{
    Eigen::Matrix<double, 5, 5> m;
    m << 1, 0, 0, 1, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

    Eigen::Matrix<double, 6, 1> imu;
    imu << 0, 0, 0, 0, 0, 0;

    double dt = 0.001; // 100Hz

    Eigen::Vector3d measured_velocity;
    measured_velocity << 1, 0, 0;

    Eigen::Matrix3d measured_velocity_covariance;
    measured_velocity_covariance << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    
    inekf::RobotState state(m);
    inekf::Correction correction;
    inekf::Propagation propagation;

    inekf::InEKF_OBJ<inekf::Propagation, inekf::Correction> inekf_obj(state, propagation, correction);


    inekf_obj.propagate_method.Propagate(imu, dt, inekf_obj.state);
    inekf_obj.correct_method.Correct(measured_velocity, measured_velocity_covariance, inekf_obj.state);
    std::cout << inekf_obj.state.getX() << std::endl;
}