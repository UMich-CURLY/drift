template<typename T>
JointStateMeasurement::JointStateMeasurement(unsigned int ENCODER_DIM)
    : Measurement(JOINT_STATE) encoder_dim_(ENCODER_DIM) {}
