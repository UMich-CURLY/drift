/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   state_estimator.h
 *  @author Tzu-Yuan Lin, Tingjun Li
 *  @brief  Header file for state estimator class
 *  @date   December 1, 2022
 **/


#ifndef LCM_HANDLER_H
#define LCM_HANDLER_H

// STL
#include <limits.h>
#include <stdlib.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <typeinfo>
// Utility types
#include "utils/cheetah_data_t.hpp"
#include 
#include "yaml-cpp/yaml.h"

// LCM related
#include <lcm/lcm-cpp.hpp>
#include "lcm-types/cheetah_inekf_lcm/leg_control_data_lcmt.hpp"
#include "lcm-types/cheetah_inekf_lcm/microstrain_lcmt.hpp"
#include "lcm-types/cheetah_inekf_lcm/reinitialization_lcmt.hpp"
#include "lcm-types/cheetah_inekf_lcm/synced_proprioceptive_lcmt.hpp"
#include "lcm-types/cheetah_inekf_lcm/wbc_test_data_t.hpp"

// Threading
#include <boost/circular_buffer.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

namespace cheetah_inekf_lcm {

class lcm_handler {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  lcm_handler(lcm::LCM* lcm, cheetah_lcm_data_t* cheetah_buffer,
              boost::mutex* cdata_mtx, bool* reinit_cmd);

  ~lcm_handler();

  void synced_msgs_lcm_callback(const lcm::ReceiveBuffer* rbuf,
                                const std::string& channel_name,
                                const synced_proprioceptive_lcmt* msg);

  void receiveReinitializeMsg(const lcm::ReceiveBuffer* rbuf,
                              const std::string& channel_name,
                              const reinitialization_lcmt* msg);

  void receiveLegControlMsg(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const leg_control_data_lcmt* msg);

  void receiveLegControlMsg_Fast(const lcm::ReceiveBuffer* rbuf,
                                 const std::string& chan,
                                 const leg_control_data_lcmt* msg);

  void receiveMicrostrainMsg(const lcm::ReceiveBuffer* rbuf,
                             const std::string& chan,
                             const microstrain_lcmt* msg);

  void receiveMicrostrainMsg_Fast(const lcm::ReceiveBuffer* rbuf,
                                  const std::string& chan,
                                  const microstrain_lcmt* msg);

  void receiveContactMsg(const lcm::ReceiveBuffer* rbuf,
                         const std::string& chan, const wbc_test_data_t* msg);

 private:
  lcm::LCM* lcm_;

  boost::mutex* cdata_mtx_;

  Eigen::Matrix<double, 12, 12> cov_encoders_;
  Eigen::Matrix<double, 6, 6> cov_prior_;

  uint64_t seq_imu_data_;
  uint64_t seq_joint_state_;
  uint64_t seq_contact_;

  cheetah_lcm_data_t* cheetah_buffer_;

  // Debugging
  bool debug_enabled_;
  bool* reinit_cmd_;
  std::ofstream kinematics_debug_;

  int64_t start_time_;          //<! the starting time of the interface
  YAML::Node config_setting;    //<! load setting config file
  YAML::Node config_inekf;      //<! load inekf config file


  /// Dimensions for sensor input:
  // leg_control_data:
  int q_dim;
  int qd_dim;
  int p_dim;
  int v_dim;
  int tau_est_dim;
  // microstrain:
  int acc_dim;
  int omega_dim;
  int quat_dim;
  int rpy_dim;
};

}    // namespace cheetah_inekf_lcm

#endif    // LCM_HANDLER_H
