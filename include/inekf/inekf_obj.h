#ifndef INEKF_INEKF_OBJ_H
#define INEKF_INEKF_OBJ_H

#include "inekf/inekf_correct.h"
#include "inekf/inekf_propagate.h"
#include "state/robot_state.h"

namespace inekf{
template<class PropagateType, class CorrectType>
struct InEKF_OBJ {
    RobotState state;
    PropagateType propagate_method;
    CorrectType correct_method;
    
    // Constructor
    InEKF_OBJ(RobotState &state, 
    PropagateType &propagate_method, 
    CorrectType &correct_method) : 
    state(state), 
    propagate_method(propagate_method), 
    correct_method(correct_method) {}
};

} // namespace inekf

#endif // INEKF_INEKF_OBJ_H