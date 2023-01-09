
set(kinematics ${PROJECT_SOURCE_DIR}/src/kinematics/mini_cheetah_kinematics.cpp)
set(minicheetah ${PROJECT_SOURCE_DIR}/src/kinematics/robots/mini_cheetah/Jp_Body_to_FrontLeftFoot.cpp
${PROJECT_SOURCE_DIR}/src/kinematics/robots/mini_cheetah/Jp_Body_to_HindLeftFoot.cpp
${PROJECT_SOURCE_DIR}/src/kinematics/robots/mini_cheetah/Jp_Body_to_FrontRightFoot.cpp
${PROJECT_SOURCE_DIR}/src/kinematics/robots/mini_cheetah/Jp_Body_to_HindRightFoot.cpp
${PROJECT_SOURCE_DIR}/src/kinematics/robots/mini_cheetah/p_Body_to_FrontLeftFoot.cpp
${PROJECT_SOURCE_DIR}/src/kinematics/robots/mini_cheetah/p_Body_to_FrontRightFoot.cpp
${PROJECT_SOURCE_DIR}/src/kinematics/robots/mini_cheetah/p_Body_to_HindLeftFoot.cpp
${PROJECT_SOURCE_DIR}/src/kinematics/robots/mini_cheetah/p_Body_to_HindRightFoot.cpp)
set(measurement ${PROJECT_SOURCE_DIR}/src/measurement/measurement.cpp
${PROJECT_SOURCE_DIR}/src/measurement/kinematics.cpp)

# Create the test binary
add_executable(${test_prefix}_cheetahkinematics_test cheetahkinematics_test.cpp ${measurement} ${kinematics} ${minicheetah})
# Link tests

target_link_libraries(
  ${test_prefix}_cheetahkinematics_test
  GTest::gtest_main
)
gtest_discover_tests(${test_prefix}_cheetahkinematics_test)