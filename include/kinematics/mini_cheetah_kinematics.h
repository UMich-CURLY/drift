
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
#include "measurement/kinematics.h"

class MiniCheetahKin : public kinematics {
 public:
  MiniCheetahKin();

  void compute_kinematics() override;
};