# Create the test binary
add_executable(${test_prefix}_cheetahkinematics_test cheetahkinematics_test.cpp)

# Link tests
target_link_libraries(
  ${test_prefix}_cheetahkinematics_test
  GTest::gtest_main
  drift
)
gtest_discover_tests(${test_prefix}_cheetahkinematics_test)