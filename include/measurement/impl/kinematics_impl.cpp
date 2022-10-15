template<typename T>
KinematicsMeasurement::KinematicsMeasurement() : Measurement(KINEMATICS) {}

void KinematicsMeasurement::set_kinematics_array(
    const inekf_msgs::KinematicsArray& kinematics) {
  kin_arr = kinematics;
}