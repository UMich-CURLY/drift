
include_directories(${PROJECT_SOURCE_DIR}/examples/include)

set(robot_state ${PROJECT_SOURCE_DIR}/src/state/robot_state.cpp)
set(se_k_3 ${PROJECT_SOURCE_DIR}/src/state/se_k_3.cpp)
set(state_estimator ${PROJECT_SOURCE_DIR}/src/state_estimator.cpp)
set(lie_group ${PROJECT_SOURCE_DIR}/src/math/lie_group.cpp)
set(base_propagation ${PROJECT_SOURCE_DIR}/src/filter/base_propagation.cpp)
set(base_correction ${PROJECT_SOURCE_DIR}/src/filter/base_correction.cpp)
set(inekf 
  ${PROJECT_SOURCE_DIR}/src/filter/inekf/correction/legged_kinematics_correction.cpp
  ${PROJECT_SOURCE_DIR}/src/filter/inekf/correction/velocity_correction.cpp
  ${PROJECT_SOURCE_DIR}/src/filter/inekf/propagation/imu_propagation.cpp
  ${PROJECT_SOURCE_DIR}/src/filter/inekf/inekf.cpp)
set(measurement ${PROJECT_SOURCE_DIR}/src/measurement/measurement.cpp 
${PROJECT_SOURCE_DIR}/src/measurement/legged_kinematics.cpp)


# Create the test binary
add_executable(legged_kinematics_correction legged_kinematics_correction.cpp ${lie_group} ${state_estimator} ${base_propagation} ${base_correction} ${inekf} ${measurement} ${robot_state} ${se_k_3}
  ${lie_group}  ${state_estimator} ${base_propagation} ${base_correction} ${inekf} ${measurement} ${robot_state} ${se_k_3})

# Link tests
target_link_libraries(
  legged_kinematics_correction
  yaml-cpp
)


