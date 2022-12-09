

#include "kinematics/robots/mini_cheetah/H_Body_to_FrontLeftFoot.h"
#include "kinematics/robots/mini_cheetah/H_Body_to_FrontRightFoot.h"
#include "kinematics/robots/mini_cheetah/H_Body_to_HindLeftFoot.h"
#include "kinematics/robots/mini_cheetah/H_Body_to_HindRightFoot.h"
#include "kinematics/robots/mini_cheetah/Jp_Body_to_FrontLeftFoot.h"
#include "kinematics/robots/mini_cheetah/Jp_Body_to_FrontRightFoot.h"
#include "kinematics/robots/mini_cheetah/Jp_Body_to_HindLeftFoot.h"
#include "kinematics/robots/mini_cheetah/Jp_Body_to_HindRightFoot.h"

#include "filter/observations.h"

class MiniCheetahKinematis {
 public:
  MiniCheetahKinematis();
  ~MiniCheetahKinematis();

  // Assumes state encoders have been updated
  void MiniCheetahKinematics::convertKinematics(CheetahState& state);
