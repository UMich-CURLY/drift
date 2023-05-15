# Create the test binary
add_executable(${test_prefix}_velocitycorrection_test velocitycorrection_test.cpp)

# Link tests
target_link_libraries(
  ${test_prefix}_velocitycorrection_test
  drift
  GTest::gtest_main
  yaml-cpp
)
gtest_discover_tests(${test_prefix}_velocitycorrection_test)
