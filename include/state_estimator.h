#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>
#include <iostream>
#include <map>
#include <memory>

#include "filter/inekf/correction/base_correction.h"
#include "filter/inekf/correction/kinematics_correction.h"
#include "filter/inekf/correction/velocity_correction.h"
#include "filter/inekf/propagation/base_propagation.h"
#include "filter/inekf/propagation/imu_propagation.h"
#include "state/robot_state.h"

using aug_map_t = std::map<int, int>;    // Augmented state map {id, aug_idx}
using namespace inekf;

class StateEstimator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name Constructors
  /// @{
  // ======================================================================
  /**
   * @brief
   *
   * @param[in] params: Noise parameters
   * @param[in] error_type: Error type of the filter
   */
  StateEstimator(NoiseParams params, ErrorType error_type);
  /// @}

  /// @name Setters
  /// @{
  // ======================================================================
  /**
   * @brief Set the initial state of the robot
   *
   * @param[in] state: Initial state of the robot
   */
  void set_state(RobotState state);
  /// @}

  /// @name Getters
  /// @{
  // ======================================================================
  /**
   * @brief Get the state of the robot
   *
   * @return RobotState: State of the robot
   */
  RobotState get_state();
  /// @}


  /// @name Propagation
  /// @{
  // ======================================================================
  /**
   * @brief Declare a propagation method, which uses imu data to propagate the
   * state of the robot
   *
   * @param[in] buffer_ptr: pointer to the imu buffer queue
   */
  template<typename imu_q_t>
  void add_imu_propagation(std::shared_ptr<imu_q_t> buffer_ptr);
  /// @}

  /// @name Correction
  /// @{
  // ======================================================================
  /**
   * @brief Declare a correction method, which uses landmark data to correct
   * the state of the robot
   *
   * @param[in] buffer_ptr: pointer to the landmark buffer queue
   */
  template<typename landmark_q_t>
  void add_landmark_correction(std::shared_ptr<landmark_q_t> buffer_ptr);

  // ======================================================================
  /**
   * @brief Declare a correction method, which uses contact data to correct
   * the state of the robot
   *
   * @param[in] buffer_ptr: pointer to the contact buffer queue
   */
  template<typename contact_q_t>
  void add_contact_correction(int contact_size,
                              std::shared_ptr<contact_q_t> buffer_ptr);

  // ======================================================================
  /**
   * @brief Declare a correction method, which uses kinematic data to correct
   * the state of the robot
   *
   * @param[in] buffer_ptr: pointer to the kinematic buffer queue
   */
  template<typename kinematic_q_t>
  void add_kinematics_correction(std::shared_ptr<kinematic_q_t> buffer_ptr);

  // ======================================================================
  /**
   * @brief Declare a correction method, which uses velocity data to correct
   * the state of the robot
   *
   * @param[in] buffer_ptr: pointer to the velocity buffer queue
   */
  template<typename velocity_q_t>
  void add_velocity_correction(std::shared_ptr<velocity_q_t> buffer_ptr,
                               const Eigen::Matrix3d& covariance);
  /// @}

  // ======================================================================
  /**
   * @brief Run the filter
   *
   * @param[in] None
   */
  void run(double dt);

 private:
  RobotState state_;
  NoiseParams params_;
  ErrorType error_type_;
  // std::shared_ptr<Correction> correction_;
  std::vector<std::shared_ptr<Correction>> corrections_;
  std::vector<aug_map_t> aug_maps;
  std::shared_ptr<Propagation> propagation_;
};    // class StateEstimator
