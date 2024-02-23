#ifndef KINEMATICS_THREE_BAR_KIN_H
#define KINEMATICS_THREE_BAR_KIN_H

#include "drift/math/lie_group.h"
#include "drift/measurement/tensegrity_kinematics.h"

namespace three_bar_tensegrity_kinematics {
enum Endcap { A, B, C, D, E, F };
}

using namespace three_bar_tensegrity_kinematics;
using namespace math;


namespace measurement::kinematics {
/**
 * @class ThreeBarTensegrityKinematics
 * @brief Three Bar Tensegrity robot specific kinematics solver and measurement container
 *
 * Derived measurement class containing Three Bar Tensegrity robot specific kinematics
 * information.
 */
class ThreeBarTensegrityKinematics : public TensegrityKinematicsMeasurement {
 public:
  /// @name Constructor
  /// @{
  /**
   * @brief Default constructor. Will generate an empty measurement.
   */
  ThreeBarTensegrityKinematics();

  /**
   * @brief Constructor with encoder and contact information
   * @param[in] encoders Joint encoder values
   * @param[in] d_encoders Joint encoder velocity values
   * @param[in] contacts Contact information
   */
  ThreeBarTensegrityKinematics(
      const Eigen::Matrix<double, Eigen::Dynamic, 1>& encoders,
      const Eigen::Matrix<double, Eigen::Dynamic, 1>& d_encoders,
      const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts);

  /// @}

  /**
   * @brief Compute kinematics and store in measurement
   */
  void ComputeKinematics() override;

  /**
   * @brief Get number of legs
   * @return Number of legs
   */
  int get_num_legs() override;

  /**
   * @brief Get initial velocity of the robot based on encoder values and
   * initial angular velocity
   * @param[in] w Initial angular velocity of the robot (rad/s)
   * @return Initial velocity
   */
  const Eigen::Vector3d get_init_velocity(const Eigen::Vector3d& w) override;
};
}    // namespace measurement::kinematics

#endif    // KINEMATICS_THREE_BAR_KIN_H