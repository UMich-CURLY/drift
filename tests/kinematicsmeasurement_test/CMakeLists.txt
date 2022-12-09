
# Create the test binary
add_executable(${test_prefix}_kinematicsmeasurement_test kinematicsmeasurement_test.cpp ${PROJECT_SOURCE_DIR}/src/measurement/measurement.cpp)
# Link tests

target_link_libraries(
  ${test_prefix}_kinematicsmeasurement_test
  GTest::gtest_main
)
gtest_discover_tests(${test_prefix}_kinematicsmeasurement_test)