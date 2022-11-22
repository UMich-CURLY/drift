#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>
#include <iostream>
#include <map>
#include <memory>

#include "inekf/inekf_correct.h"
#include "inekf/inekf_propagate.h"
#include "state/robot_state.h"

namespace inekf {
using aug_map_t = std::map<int, int>;    // Augmented state map {id, aug_idx}
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
   * @brief Set imu data buffer
   *
   * @template T: Type of the data buffer
   * @param[in] topic_name: Name of the sensor's topic
   * @return std::shared_ptr<imu_q_t>: Pointer to the imu data buffer
   */
  template<typename imu_q_t>
  std::shared_ptr<imu_q_t> add_imu_subscriber(std::string topic_name);

  // ======================================================================
  /**
   * @brief Set landmark data buffer
   *
   * @template T: Type of the data buffer
   * @param[in] topic_name: Name of the sensor's topic
   * @return std::shared_ptr<landmark_q_t>: Pointer to the landmark data buffer
   */
  template<typename landmark_q_t>
  std::shared_ptr<landmark_q_t> add_landmark_subscriber(std::string topic_name);

  // ======================================================================
  /**
   * @brief Set contact data buffer
   *
   * @template T: Type of the data buffer
   * @param[in] topic_name: Name of the sensor's topic
   * @return std::shared_ptr<contact_q_t>: Pointer to the contact data buffer
   */
  template<typename contact_q_t>
  std::shared_ptr<contact_q_t> add_contact_subscriber(std::string topic_name);

  // ======================================================================
  /**
   * @brief Set kinematic data buffer
   *
   * @template T: Type of the data buffer
   * @param[in] topic_name: Name of the sensor's topic
   * @return std::shared_ptr<kinematic_q_t>: Pointer to the kinematic data
   * buffer
   */
  template<typename kinematic_q_t>
  std::shared_ptr<kinematic_q_t> add_kinametic_subscriber(
      std::string topic_name);


  // ======================================================================
  /**
   * @brief Set velocity data buffer
   *
   * @template T: Type of the data buffer
   * @param[in] topic_name: Name of the sensor's topic
   * @return std::shared_ptr<velocity_q_t>: Pointer to the velocity data
   * buffer
   */
  template<typename velocity_q_t>
  std::shared_ptr<velocity_q_t> add_velocity_subscriber(std::string topic_name);

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

  void add_aug_map(aug_map_t aug_map);

  template<typename imu_q_t>
  void add_imu_propagation(std::shared_ptr<imu_q_t> buffer_ptr);

  template<typename landmark_q_t>
  void add_landmark_correction(std::shared_ptr<landmark_q_t> buffer_ptr);

  template<typename contact_q_t>
  void add_contact_correction(int contact_size,
                              std::shared_ptr<contact_q_t> buffer_ptr);

  template<typename kinematic_q_t>
  void add_kinematics_correction(std::shared_ptr<kinematic_q_t> buffer_ptr);

  template<typename velocity_q_t>
  void add_velocity_correction(std::shared_ptr<velocity_q_t> buffer_ptr);

  void run();

 private:
  RobotState state_;
  NoiseParams params_;
  ErrorType error_type_;
  std::vector<std::shared_ptr<Correction>> corrections;
  std::vector<aug_map_t> aug_maps;
  std::shared_ptr<Propagation> propagation;
};    // class StateEstimator
}    // namespace inekf
