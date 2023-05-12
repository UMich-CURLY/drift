# State Estimator Tutorial (Adding a custom legged kinematics class)

This tutorial will guide you through adding a custom legged kinematics class to our library to enable legged kinematics correction functionality on a unique legged platform. The class allows computation of the foot-frame body displacement given the robot joint variables. The DRIFT library uses this definition to subsequently integrate foot contact events to compute localization of the body in the local world-frame. The DRIFT library comes with kinematics definitions for the MIT Mini Cheetah and the Unitree Go1 platforms. For any other legged platform, a custom class will be necessary containing the appropriate kinematic chain forward transform and Jacobian definitions. 

### Step 1: Generate Kinematics Definition Header and Source Files
Users must to generate their own kinematics files. For this, we used the [Fast Robot Optimization and Simulation Toolkit (FROST) Library](http://ayonga.github.io/frost-dev/).

The kinematics definition header files should be placed under an appropriately named folder under the `drift/include/kinematics/robots` directory, and source files should be placed under an appropriately named folder under the `drift/src/kinematics/robots` directory, similar to the robots/mini_cheetah folders.

### Step 2: Create Custom Kinematics Class Header and Source Files
Users can create a new derived LeggedKinematicsMeasurement class by following the format of the existing `drift/include/kinematics/mini_cheetah_kinematics.h` header file. Let's take the `mini_cheetah_kinematics.h` header file as an example. The full file looks like:
```cpp
#ifndef MC_KIN_H
#define MC_KIN_H

#include "drift/kinematics/robots/mini_cheetah/Jp_Body_to_FrontLeftFoot.h"
#include "drift/kinematics/robots/mini_cheetah/Jp_Body_to_FrontRightFoot.h"
#include "drift/kinematics/robots/mini_cheetah/Jp_Body_to_HindLeftFoot.h"
#include "drift/kinematics/robots/mini_cheetah/Jp_Body_to_HindRightFoot.h"
#include "drift/kinematics/robots/mini_cheetah/p_Body_to_FrontLeftFoot.h"
#include "drift/kinematics/robots/mini_cheetah/p_Body_to_FrontRightFoot.h"
#include "drift/kinematics/robots/mini_cheetah/p_Body_to_HindLeftFoot.h"
#include "drift/kinematics/robots/mini_cheetah/p_Body_to_HindRightFoot.h"
#include "drift/math/lie_group.h"
#include "drift/measurement/legged_kinematics.h"

namespace mini_cheetah_kinematics {
enum Leg { FR, FL, HR, HL };
}

using namespace mini_cheetah_kinematics;
using namespace math;


namespace measurement::kinematics {
/**
 * @class MiniCheetahKinematics
 * @brief Mini Cheetah specific kinematics solver and measurement container
 *
 * Derived measurement class containing Mini Cheetah specific kinematics
 * information.
 */
class MiniCheetahKinematics : public LeggedKinematicsMeasurement {
 public:
  /// @name Constructor
  /// @{
  /**
   * @brief Default constructor. Will generate an empty measurement.
   */
  MiniCheetahKinematics();

  /**
   * @brief Constructor with encoder and contact information
   * @param[in] encoders Joint encoder values
   * @param[in] d_encoders Joint encoder velocity values
   * @param[in] contacts Contact information
   */
  MiniCheetahKinematics(
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

#endif    // MC_KIN_H
```

Let's go through the code step by step.

#### 2.1 Define guards and include necessary libraries
We need to include necessary libraries along with the header files of kinematic position transforms and Jacobian definitions:
```cpp
#ifndef MC_KIN_H  //define guards
#define MC_KIN_H

#include "drift/kinematics/robots/mini_cheetah/Jp_Body_to_FrontLeftFoot.h"  // Jacobian Definitions
#include "drift/kinematics/robots/mini_cheetah/Jp_Body_to_FrontRightFoot.h"
#include "drift/kinematics/robots/mini_cheetah/Jp_Body_to_HindLeftFoot.h"
#include "drift/kinematics/robots/mini_cheetah/Jp_Body_to_HindRightFoot.h"
#include "drift/kinematics/robots/mini_cheetah/p_Body_to_FrontLeftFoot.h"   // position transforms
#include "drift/kinematics/robots/mini_cheetah/p_Body_to_FrontRightFoot.h"
#include "drift/kinematics/robots/mini_cheetah/p_Body_to_HindLeftFoot.h"
#include "drift/kinematics/robots/mini_cheetah/p_Body_to_HindRightFoot.h"

#include "drift/math/lie_group.h"                 // lie group header must be included
#include "drift/measurement/legged_kinematics.h"  // legged kinematics inherited class header must be included
```

#### 2.2 Namespace and leg enumerators
Create custom namespace, in this example we use the mini_cheetah_kinematics namespace.
In cases with >2 legs, enumeration will make indexing matricies easier. Indexing should match the contact definitions matrix.
```cpp
namespace mini_cheetah_kinematics {
enum Leg { FR, FL, HR, HL };  // leg definitions enumerated according to some intuitive scheme i.e. FR for "front right"
}

using namespace mini_cheetah_kinematics;
using namespace math;


namespace measurement::kinematics {  // measurement::kinematics namespace will be used for the class
```

### 2.3 Class definition
Next, create the class derived from LeggedKinematicsMeasurement with the following constructors and member functions:
```cpp
/**
 * @class MiniCheetahKinematics
 * @brief Mini Cheetah specific kinematics solver and measurement container
 *
 * Derived measurement class containing Mini Cheetah specific kinematics
 * information.
 */
class MiniCheetahKinematics : public LeggedKinematicsMeasurement {
 public:
  /// @name Constructor
  /// @{
  /**
   * @brief Default constructor. Will generate an empty measurement.
   */
  MiniCheetahKinematics();

  /**
   * @brief Constructor with encoder and contact information
   * @param[in] encoders Joint encoder values
   * @param[in] d_encoders Joint encoder velocity values
   * @param[in] contacts Contact information
   */
  MiniCheetahKinematics(
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

#endif    // MC_KIN_H define guard close
```
