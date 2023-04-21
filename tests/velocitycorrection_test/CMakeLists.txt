
set(robot_state ${PROJECT_SOURCE_DIR}/src/state/robot_state.cpp)
set(inekf_estimator ${PROJECT_SOURCE_DIR}/src/estimator/inekf_estimator.cpp)
set(math ${PROJECT_SOURCE_DIR}/src/math/lie_group.cpp
  ${PROJECT_SOURCE_DIR}/src/math/se_k_3.cpp)
set(base_propagation ${PROJECT_SOURCE_DIR}/src/filter/base_propagation.cpp)
set(base_correction ${PROJECT_SOURCE_DIR}/src/filter/base_correction.cpp)
set(inekf
  ${PROJECT_SOURCE_DIR}/src/filter/inekf/correction/legged_kinematics_correction.cpp
  ${PROJECT_SOURCE_DIR}/src/filter/inekf/correction/velocity_correction.cpp
  ${PROJECT_SOURCE_DIR}/src/filter/inekf/propagation/imu_propagation.cpp
  ${PROJECT_SOURCE_DIR}/src/filter/inekf/inekf.cpp)
set(measurement ${PROJECT_SOURCE_DIR}/src/measurement/measurement.cpp
  ${PROJECT_SOURCE_DIR}/src/measurement/legged_kinematics.cpp)
set(imu_filter ${PROJECT_SOURCE_DIR}/src/imu_filter/imu_ang_vel_ekf.cpp)

# Create the test binary
add_executable(${test_prefix}_velocitycorrection_test velocitycorrection_test.cpp ${math} ${imu_filter} ${inekf_estimator} ${base_propagation} ${base_correction} ${inekf} ${measurement} ${robot_state}
  ${math} ${imu_filter} ${inekf_estimator} ${base_propagation} ${base_correction} ${inekf} ${measurement} ${robot_state})

# Link tests
target_link_libraries(
  ${test_prefix}_velocitycorrection_test
  GTest::gtest_main
  yaml-cpp
)
gtest_discover_tests(${test_prefix}_velocitycorrection_test)
