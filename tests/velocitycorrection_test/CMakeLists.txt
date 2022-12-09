
set(robot_state ${PROJECT_SOURCE_DIR}/src/state/robot_state.cpp)
set(se_k_3 ${PROJECT_SOURCE_DIR}/src/state/se_k_3.cpp)
set(state_estimator ${PROJECT_SOURCE_DIR}/src/state_estimator.cpp)
set(lie_group ${PROJECT_SOURCE_DIR}/src/math/lie_group.cpp)
set(noise_params ${PROJECT_SOURCE_DIR}/src/filter/noise_params.cpp)
set(base_propagation ${PROJECT_SOURCE_DIR}/src/filter/base_propagation.cpp)
set(base_correction ${PROJECT_SOURCE_DIR}/src/filter/base_correction.cpp)
set(inekf ${PROJECT_SOURCE_DIR}/src/filter/inekf/correction/kinematics_correction.cpp
  ${PROJECT_SOURCE_DIR}/src/filter/inekf/correction/velocity_correction.cpp
  ${PROJECT_SOURCE_DIR}/src/filter/inekf/propagation/imu_propagation.cpp
  ${PROJECT_SOURCE_DIR}/src/filter/inekf/inekf.cpp)
set(measurement ${PROJECT_SOURCE_DIR}/src/measurement/measurement.cpp)

# Create the test binary
add_executable(${test_prefix}_velocitycorrection_test velocitycorrection_test.cpp ${lie_group} ${noise_params} ${state_estimator} ${base_propagation} ${base_correction} ${inekf} ${measurement} ${robot_state} ${se_k_3}
  ${lie_group} ${noise_params} ${state_estimator} ${base_propagation} ${base_correction} ${inekf} ${measurement} ${robot_state} ${se_k_3})

# Link tests
target_link_libraries(
    ${test_prefix}_velocitycorrection_test
    GTest::gtest_main
)
gtest_discover_tests(${test_prefix}_velocitycorrection_test)


