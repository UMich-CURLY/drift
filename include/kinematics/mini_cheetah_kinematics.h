
#include "kinematics/robots/mini_cheetah/Jp_Body_to_FrontLeftFoot.h"
#include "kinematics/robots/mini_cheetah/Jp_Body_to_FrontRightFoot.h"
#include "kinematics/robots/mini_cheetah/Jp_Body_to_HindLeftFoot.h"
#include "kinematics/robots/mini_cheetah/Jp_Body_to_HindRightFoot.h"
#include "kinematics/robots/mini_cheetah/p_Body_to_FrontLeftFoot.h"
#include "kinematics/robots/mini_cheetah/p_Body_to_FrontRightFoot.h"
#include "kinematics/robots/mini_cheetah/p_Body_to_HindLeftFoot.h"
#include "kinematics/robots/mini_cheetah/p_Body_to_HindRightFoot.h"
#include "measurement/contact.h"
#include "measurement/joint_state.h"
#include "measurement/legged_kinematics.h"

enum Leg { FR, FL, HL, HR };

class MiniCheetahKin : public legged_kinematics {
 public:
  MiniCheetahKin();
  MiniCheetahKin(const Eigen::Matrix<double, Eigen::Dynamic, 1>& encoders,
                 const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts);

  void compute_kinematics() override;
};