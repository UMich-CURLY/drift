
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

#include "communication/ros_subscriber.h"
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


int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_state_est");


  // ros handle handles the start/shutdown for us
  ros::NodeHandle nh;

  ros_wrapper::ROSSubscriber ros_sub(&nh);

  inekf::ErrorType error_type = RightInvariant;
  StateEstimator state_estimator(params, error_type);
  std::cout << "Before: " << std::endl;
  std::cout << state_estimator.get_state().get_X() << std::endl;

  // Add propagation and correction methods

  auto imu_data_buffer_ptr = ros_sub.AddIMUSubscriber("/gx5_0/imu/data");
  state_estimator.add_imu_propagation(imu_data_buffer_ptr, false);


  state_estimator::init_bias(){};
  state_estimator::init_state();
  auto velocity_data_buffer_ptr
      = ros_sub.AddDifferentialDriveVelocitySubscriber("/joint_states");
  state_estimator.add_velocity_correction(velocity_data_buffer_ptr);


  ros_sub.StartSubscribingThread();

  std::vector<Eigen::Matrix<double, 5, 5>> expect_X;
  Eigen::Matrix<double, 5, 5> X = Eigen::Matrix<double, 5, 5>::Identity();
  X(0, 4) = 0.5;
  expect_X.push_back(X);

  Eigen::Matrix2d R;
  R << 0, -1, 1, 0;
  X.block<2, 2>(0, 0) = R;
  expect_X.push_back(X);

  X(1, 3) = 1;
  X(1, 4) = 0.5;
  expect_X.push_back(X);

  for (int i = 0; i < 3; i++) {
    state_estimator.run();
    std::cout << "After: " << std::endl;
    std::cout << state_estimator.get_state().get_X() << std::endl;
    auto est_state = state_estimator.get_state().get_X();
    for (int j = 0; j < 5; j++) {
      for (int k = 0; k < 5; k++) {
        EXPECT_NEAR(est_state(j, k), expect_X[i](j, k), 1e-6);
      }
    }
  }
}