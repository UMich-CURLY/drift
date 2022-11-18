#ifndef ROBOT_DATA_H
#define ROBOT_DATA_H

/* Robot Data Structures */
#include "measurement/contact.h"
#include "measurement/imu.h"
#include "measurement/joint_state.h"

#include <memory>
#include <mutex>
#include <queue>
#include <stack>
#include <thread>


struct robot_data_t {
  robot_data_t() {}

  std::mutex imu_mutex;
  std::mutex joint_state_mutex;
  std::mutex vel_mutex;
  std::queue<std::shared_ptr<ImuMeasurement<double>>> imu_q;
  // Use vector like a stack, using vector to enable O(1) clear operation
  std::queue<std::shared_ptr<JointStateMeasurement>> joint_state_q;
};

#endif    // ROBOT_DATA_H
