
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

class MiniCheetahKin : public KinematicsMeasurement<double> {
 public:
  MiniCheetahKin();

  void compute_kinematics() override;

  void set_joint_state(const JointStateMeasurement<12, double>& js);

  void set_contact_state(const ContactMeasurement<4>& ct);

  JointStateMeasurement<12, double> get_joint_state() const;

  ContactMeasurement<4> get_contact_state() const;

 private:
  ContactMeasurement<4> contact_;
  JointStateMeasurement<12, double> js_;
};